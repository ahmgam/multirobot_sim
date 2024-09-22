#!/usr/bin/env python3
from multirobot_sim.srv import GetBCRecords,SubmitTransaction,AddGoal,GetBCRecordsRequest,SubmitTransactionRequest,AddGoalResponse
from rospy import ServiceProxy,Service,Publisher,Subscriber, loginfo, wait_for_message,init_node,get_namespace, get_param,ROSInterruptException, is_shutdown
import json
from actionlib import SimpleActionClient,GoalStatus
from datetime import datetime
import numpy as np
from std_srvs.srv import Trigger
from multirobot_sim.msg import NavigationActionAction,NavigationActionGoal
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped,Point
from nav_msgs.srv import GetMap
from path_planning import AStar,RTT
from std_msgs.msg import String
from time import mktime
import uuid
#default value of state update interval
UPDATE_INTERVAL = 10
class Planner:
    def __init__(self,odom_topic,algorithm=None):
        self.path = Path()
        self.goal=None
        self.map = self.getTheMap()
        self.modified_map = self.map
        self.odom_topic= odom_topic
        odom = self.getOdomMsg()
        self.start = (odom.pose.pose.position.x,odom.pose.pose.position.y)
        loginfo(f"map size is {len(self.map.data)}")
        self.grid = self.formatGrid(self.map)
        loginfo(f"planner:Grid formatted, shape is {self.grid.shape}")
        self.gridInfo = self.formatGridInfo(self.map.info)
        loginfo(f"planner:Grid info formatted, info is {self.gridInfo}")
        self.algorithm = self.defineAlgorithm(algorithm)            
        loginfo(f"planner:Algorithm defined, algorithm is {type(self.algorithm)}")

    def getTheMap(self,mapService='/static_map'):
        #wait for map service
        loginfo("simple_controller:Waiting for map service")
        serv = ServiceProxy(mapService, GetMap)
        serv.wait_for_service()
        map = serv().map
        return map
    
    def getOdomMsg(self):
        odom = wait_for_message(self.odom_topic, Odometry, timeout=5.0)
        return odom
    def defineAlgorithm(self,algorithm):
        if algorithm is None:
            return None
        if algorithm == "AStar":
            return AStar()
        if algorithm == "RTT":
            return RTT()

    def formatGrid(self,grid):
        #convert grid to 2d array
        return np.array(grid.data).reshape(grid.info.height,grid.info.width)

    def formatGridInfo(self,info):
        return {
            "width":info.width,
            "height":info.height,
            "resolution":info.resolution,
            "origin":(info.origin.position.x,info.origin.position.y)
        }
    
    def posToGrid(self,pos):
        x,y = pos[0],pos[1]
        x = int((x - self.gridInfo["origin"][0])/self.gridInfo["resolution"])
        y = int((y - self.gridInfo["origin"][1])/self.gridInfo["resolution"])
        return (x,y)

    def gridToPos(self,grid):
        x,y = grid[0],grid[1]
        x = x*self.gridInfo["resolution"] + self.gridInfo["origin"][0]
        y = y*self.gridInfo["resolution"] + self.gridInfo["origin"][1]
        return (x,y)

    def parsePath(self,path):
        formattedPath = Path()
        formattedPath.header.frame_id = "map"
        loginfo(f"planner: path is {path}@@@")
        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = node[0]* self.gridInfo["resolution"] + self.gridInfo["origin"][0]
            pose.pose.position.y = node[1]*self.gridInfo["resolution"] + self.gridInfo["origin"][1]
            pose.pose.position.z = 0
            formattedPath.poses.append(pose)
        return formattedPath

    def generate_line_points(self,p1, p2):
        """Generate a list of points in a line from p1 to p2 using Bresenham's line algorithm."""
        x1, y1 = p1
        x2, y2 = p2
        points = []
        is_steep = abs(y2 - y1) > abs(x2 - x1)
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        dx = x2 - x1
        dy = y2 - y1
        error = int(dx / 2.0)
        y = y1
        if y1 < y2:
            ystep = 1
        else:
            ystep = -1
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points
    def mergePathsWithMap(self,other_paths):
        for path in other_paths:
            for i in range(len(path)-1):
                p1 = path[i]
                p2 = path[i+1]
                points = self.generate_line_points(p1,p2)
                for point in points:
                    self.modified_map[point[0],point[1]] = 100

    def clearModifiedMap(self):
        self.modified_map = self.map

    def plan(self,other_paths=None):
        if self.algorithm is None:
            if self.goal is None:
                return
            self.path = [self.goal]
        else :
            
            self.algorithm.setStart(self.posToGrid(self.start))
            loginfo(f"planner:Start set to {self.algorithm.start}")
            self.algorithm.setGoal(self.posToGrid(self.goal))
            loginfo(f"planner:Goal set to {self.algorithm.goal}")
            if other_paths is None:
                self.algorithm.setMap(self.grid,self.gridInfo)
                self.algorithm.plan()
            else:
                self.mergePathsWithMap(other_paths)
                self.algorithm.setMap(self.modified_map,self.gridInfo)
                self.algorithm.plan()
                self.clearModifiedMap()
            
            self.path =self.parsePath(self.algorithm.path)

    def setGoal(self, x, y,z):
        self.goal = (x, y,z)
        loginfo(f"planner:Goal set to {self.goal}")


class TaskAllocationManager:
    def __init__(self,planningAlgorithm=None):
        self.node_id,self.node_type,self.odom_topic,self.update_interval = self.getParameters()
        loginfo(f"{self.node_id}: Task_allocator: Initializing")
        self.node = init_node('task_allocator', anonymous=True)
        loginfo(f"{self.node_id}: Task_allocator: Initializing parameters")
        self.robots = {}
        self.targets = {}
        self.tasks = {}
        self.buffered_hashes = []
        self.paths = {}
        self.pos_x = None
        self.pos_y = None
        self.idle= {self.node_id:True}
        self.waiting_message = None
        self.ongoing_task = None
        self.last_id = 1
        loginfo(f"{self.node_id}: Task_allocator: Initializing services")
        self.get_blockchain_records = ServiceProxy(f'/{self.node_id}/roschain/get_records',GetBCRecords)
        self.get_blockchain_records.wait_for_service(timeout=25)
        loginfo(f"{self.node_id}: Task_allocator: Initializing get_status service client")
        self.chain_status = ServiceProxy(f'/{self.node_id}/roschain/is_ready',Trigger)
        self.chain_status.wait_for_service(timeout=25)
        loginfo(f"{self.node_id}: Task_allocator: Initializing get_records service client")
        self.submit_message = ServiceProxy(f'/{self.node_id}/roschain/submit_message',SubmitTransaction)
        self.submit_message.wait_for_service(timeout=25)
        loginfo(f"{self.node_id}: Task_allocator: Initializing submit_message service client")
        self.target_discovery = Service(f'/{self.node_id}/add_goal',AddGoal,lambda data: self.add_goal(data))
        loginfo(f"{self.node_id}: Task_allocator: Initializing add_goal service")
        self.navigation_client = SimpleActionClient('navigator',NavigationActionAction)
        loginfo(f"{self.node_id}: Task_allocator: Initializing navigation action client")
        self.planner = Planner(self.odom_topic,planningAlgorithm)
        self.last_state = datetime.now()
        self.path_publisher = Publisher(f'/{self.node_id}/path',Path,queue_size=1)
        loginfo(f"{self.node_id}: Task_allocator: Initializing path publisher")
        #self.get_blockchain_records = ServiceProxy('get_blockchain_records')
        self.log_publisher = Publisher(f"/{self.node_id}/connector/send_log", String, queue_size=10)
        #define goal found subscriber
        self.goal_found = Subscriber(f"/{self.node_id}/goal_found",String,self.goal_found_callback)

    def goal_found_callback(self,data):
        data = data.data
        splitted = data.split(",")
        if len(splitted) != 4:
            loginfo(f"{self.node_id}: Task_allocator: Invalid goal found message, it should be in the format x,y,needed_uavs,needed_ugvs")
            return
        x,y,needed_uavs,needed_ugvs = float(splitted[0]),float(splitted[1]),int(splitted[2]),int(splitted[3])
        goal_uuid = str(uuid.uuid4())
        self.add_goal(x,y,needed_uavs,needed_ugvs,goal_uuid)
        self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:publishing,target,started")
        
    def getParameters(self):
        loginfo(f"task_allocator: getting namespace")
        ns = get_namespace()
        try :
            node_id= get_param(f'/{ns}/task_allocation/node_id') # node_name/argsname
            loginfo(f"task_allocator:Getting node_id argument, and got : {node_id}")

        except ROSInterruptException:
            raise ROSInterruptException("Invalid arguments : node_id")

        try :
            node_type= get_param(f'/{ns}/task_allocation/node_type') # node_name/argsname
            loginfo(f"task_allocator:Getting node_type argument, and got : {node_type}")

        except ROSInterruptException:
            raise ROSInterruptException("Invalid arguments : node_type")
        
        
        try :
            odom_topic= get_param(f'/{ns}/task_allocation/odom_topic') # node_name/argsname
            loginfo(f"task_allocator:Getting odom_topic argument, and got : {odom_topic}")

        except ROSInterruptException:
            raise ROSInterruptException("Invalid arguments : odom_topic")
        
        try :
            update_interval= get_param(f'/{ns}/task_allocation/update_interval',UPDATE_INTERVAL) # node_name/argsname
            loginfo(f"task_allocator:Getting update_interval argument, and got : {update_interval}")

        except ROSInterruptException:
            raise ROSInterruptException("Invalid arguments : update_interval")
        
        return node_id,node_type,odom_topic,update_interval
    def add_goal(self,x,y,needed_uavs,needed_ugvs,uuid):
        msg = json.dumps(
        {
          "node_id":self.node_id,
          "uuid":uuid,
          "timecreated":datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
          "pos_x":x,
          "pos_y":y,
          "needed_uav":needed_uavs,
          "needed_ugv":needed_ugvs
          
        })
        self.submit_message('targets',msg)
    def update_position(self):
        odom = wait_for_message(self.odom_topic, Odometry)
        self.pos_x = odom.pose.pose.position.x
        self.pos_y = odom.pose.pose.position.y

    def sync_records(self):
        #get new records from blockchain service
        records = self.get_blockchain_records(self.last_id)
        for record in records.transactions:
            record = json.loads(record)
            #record = list(record.values())[0]
            self.process_record(record)
         
    def handle_record(self,record):
        loginfo(f"{self.node_id}: Task_allocator: Processing record of type {record['meta']['item_table']}")
        if record['data']['node_id'] not in self.idle.keys():
                self.idle[record['data']['node_id']] = True
        if record['meta']['item_table'] == 'states':
            self.robots[record['data']['node_id']] = record['data']
        if record['meta']['item_table'] == 'targets':
            self.targets[record['data']['uuid']] = record['data']
            self.tasks[record['data']['uuid']]= []
            self.paths[record['data']['uuid']] = {}
            self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:received,{record['data']['node_id']},target,{record['data']['uuid']}")
        if record['meta']['item_table'] == 'task_records':
            data = record['data']
            print(f"task record is {data} and ")
            if data['record_type'] == 'commit':
                self.idle[data['node_id']] = False
            else:
                self.idle[data['node_id']] = True
            if data['target_id'] in self.tasks.keys():
                self.tasks[data['target_id']].append(data)
            else:
                self.tasks[data['target_id']] = [data]
            self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:received,{data['node_id']},{data['record_type']},{data['target_id']}")
            if self.is_task_completed(data['target_id']):
                self.clear_task(data['target_id'])

        if record['meta']['item_table'] == 'paths':
            data = record['data']
            data['path_points'] = json.loads(data['path_points'])
            target_id = data['target_id']
            if target_id in self.paths.keys():
                self.paths[target_id][data['node_id']] = data
                
            else:
                self.paths[target_id] = {data['node_id']:data}
            self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:received,{data['node_id']},path,{data['target_id']}")
    def process_record(self,record):
        if record['committed'] == False:
            if record['meta']['hash'] not in self.buffered_hashes:
                self.buffered_hashes.append(record['meta']['hash'])
                self.handle_record(record)
        else:
            if record['meta']['hash'] in self.buffered_hashes:
                #self.buffered_hashes.remove(record['meta']['hash'])
                pass
            else:
                self.handle_record(record)
            #update last id 
            self.last_id = record['meta']['id'] + 1
        if self.is_in_waiting(record['data'],record['meta']['item_table']):
            self.waiting_message = None

    def is_task_fully_committed(self,task_id):
        #check all records in task
        task = self.tasks[task_id]
        req_uav = int(self.targets[task_id]['needed_uav'])
        req_ugv = int(self.targets[task_id]['needed_ugv'])
        committed_uav = 0
        committed_ugv = 0
        for record in task:
            if record['record_type'] == 'commit':
                if self.robots[record['node_id']]['node_type'] == 'uav':
                    committed_uav += 1
                if self.robots[record['node_id']]['node_type'] == 'ugv':
                    committed_ugv += 1
        if committed_ugv == req_ugv and committed_uav == req_uav:
            return True
        else:
            return False

    def is_task_planned(self,task_id):
        #get all paths records for a task
        paths = self.paths[task_id]
        req_uav = int(self.targets[task_id]['needed_uav'])
        req_ugv = int(self.targets[task_id]['needed_ugv'])
        planned_uav = {}
        planned_ugv = {}
        for node_id,path in paths[task_id].items():
            if path['node_type'] == 'uav':
                planned_uav[node_id] = path
            if path['node_type'] == 'ugv':
                planned_ugv[node_id] = path

        if len(planned_uav.keys()) != req_uav or len(planned_ugv.keys()) != req_ugv:
            return False
        #check paths conflicts 
        return self.check_conflict(task_id)
            
    def is_task_completed(self,task_id):
        #check all records in task
        task = self.tasks[task_id]
        req_robots = int(self.targets[task[0]['target_id']]['needed_uav']) + int(self.targets[task[0]['target_id']]['needed_ugv'])
        completed = 0
        for record in task:
            if record['record_type'] == 'complete':
                completed += 1
        if completed >= req_robots:
            return True
        else:
            return False

    def clear_task(self,task_id):
        #get task
        if task_id in self.tasks.keys():
            del self.tasks[task_id] 
        if task_id in self.targets.keys():
            del self.targets[task_id]
        if task_id in self.paths.keys():
            del self.paths[task_id]


    def get_target_best_candidates(self,target_id):
        #get task details 
        goal = self.targets[target_id]['pos_x'],self.targets[target_id]['pos_y']
        distances = []
        for robot in self.robots.values():
            if self.is_robot_idle(robot['node_id']):
                robot_pos = robot['pos_x'],robot['pos_y']
                distances.append({
                    "node_id":robot['node_id'],
                    "node_type":robot['node_type'],
                    "distance":self.caluculate_distance(robot_pos,goal)
                    })
        #sort distances
        distances.sort(key=lambda x: x["distance"], reverse=True)
        #get needed uavs and ugvs 
        needed_uav = int(self.targets[target_id]['needed_uav'])
        needed_ugv = int(self.targets[target_id]['needed_ugv'])
        robots = []
        #get best candidates
        for distance in distances:
            if distance["node_type"] == "uav":
                if needed_uav > 0:
                    robots.append(distance)
                    needed_uav -= 1
            if distance["node_type"] == "ugv":
                if needed_ugv > 0:
                    robots.append(distance)
                    needed_ugv -= 1
        return robots

    def get_best_target(self):
        target_id = None
        best_targets = []
        for id,target in self.targets.items():
            robots = self.get_target_best_candidates(id)
            for robot in robots:
                if robot['node_id'] == self.node_id :
                    best_targets.append({
                        "target_id":target['uuid'],
                        "node_id":robot['node_id'],
                        "distance":robot['distance']
                    })

        #sort best targets
        best_targets.sort(key=lambda x: x['distance'], reverse=True)
        if len(best_targets) > 0:
            target_id = best_targets[0]['target_id']
        return target_id

    def is_task_executable(self,target_id):
        #get needed uav and ugv
        needed_uav = int(self.targets[target_id]['needed_uav'])
        needed_ugv = int(self.targets[target_id]['needed_ugv'])
        #get all idle robots
        for path in self.paths[target_id].values():
                if path['node_type'] == 'uav':
                    needed_uav -= 1
                if path['node_type'] == 'ugv':
                    needed_ugv -= 1
        
        if needed_uav <= 0 and needed_ugv <= 0:
            return True
        else:
            return False

        
    def caluculate_distance(self,robot_pos,goal):
        #calculate euclidean distance
        return np.sqrt((goal[0] - robot_pos[0])**2 + (goal[1] - robot_pos[1])**2)

    def is_robot_idle(self,robot_id):
        return self.idle[robot_id]
    
    def commmit_to_target(self,target):
        #prepare payload
        payload = {
            'node_id':self.node_id,
            'target_id':str(target),
            'record_type':'commit',
            'timecreated':datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.add_waiting_message(payload,'task_records')
        self.submit_message('task_records',json.dumps(payload))
        self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:publishing,commit,{str(target)}")

    def add_waiting_message(self,message,msg_type):
        self.waiting_message = {
            "type":msg_type,
            "message":message
        }
    
    def format_path(self,path):
        #convert path from list of tuples to list of points
        points = []
        for point in path.poses:
            points.append([point.pose.position.x,point.pose.position.y])
        return points
    
    def start_task(self,path):
        points = []
        for point in path:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0
            points.append(p)    
        goal = NavigationActionGoal(points=points)
        self.navigation_client.send_goal(goal)

    def visualize_path(self,path):
        self.path_publisher.publish(path)
        
    def check_conflict(self,target_id):
        all_paths = list(self.paths[target_id].values())
        conflicted_paths = []
        while len(all_paths) > 0:
            first_path = all_paths[0]
            for i in range(1,len(all_paths)):
                if self.are_paths_intersection(first_path["path_points"],all_paths[i]["path_points"]):
                    conflicted_paths.append((first_path["node_id"],all_paths[i]["node_id"]))
                    
            all_paths.remove(first_path)
        return conflicted_paths

    def are_paths_intersection(self,waypoints1,waypoints2):
        if len(list(set(waypoints1).intersection(waypoints2))) != 0:
            return True
        ccw = lambda A,B,C: (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        do_segments_intersect= lambda A,B,C,D: ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
        for i in range(len(waypoints1) - 1):
            for j in range(len(waypoints2) - 1):
                if do_segments_intersect(waypoints1[i], waypoints1[i + 1], waypoints2[j], waypoints2[j + 1]):
                    return True
        return False 

    def is_committed(self):
        #check if current robot has committed to a target
        for task in self.tasks.values():
            for record in task:
                if record['node_id'] == self.node_id and record['record_type'] == 'commit':
                    return True,record
        return False,None
    
    
    def is_in_waiting(self,message=None,msg_type=None):
        if self.waiting_message == None:
            return False
        #check if message is in waiting
        if msg_type == "task_records":
            loginfo(f"{self.node_id}: Task_allocator: Waiting : {self.waiting_message} andn got {message} for task_records mf@@@")
        if msg_type == 'paths':
            loginfo(f"{self.node_id}: Task_allocator: Waiting : {self.waiting_message} andn got {message} for paths mf@@@")
        if message!= None and msg_type != None:
            if message.get('id') != None:
                message.pop('id')
            if message["node_id"] != self.node_id or msg_type != self.waiting_message['type']:
                return False
            loginfo(f"{self.node_id}: Task_allocator: Waiting for {msg_type} message@@@")
            return True
        return True
    def send_complete_message(self):
        #send complete message to blockchain
        payload = {
            'node_id':self.node_id,
            'record_type':'complete',
            'target_id':self.ongoing_task,
            'timecreated':datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        self.add_waiting_message(payload,'task_records')
        #msg = SubmitTransaction(table_name='task_records',message=json.dumps(payload))
        self.submit_message('task_records',json.dumps(payload))
        self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:publishing,complete,{self.ongoing_task}")
    def check_ongoing_task(self):
        #check the status of ongoing task
        if self.navigation_client.get_state() == GoalStatus.SUCCEEDED:
            #set robot state to idle
            self.idle[self.node_id] = True
            #send complete message to blockchain
            self.send_complete_message()
            self.ongoing_task = None
        return

    def is_path_submitted(self,target_id):
        if self.paths[target_id].get(self.node_id) != None:
            return True,self.paths[target_id][self.node_id]["path_points"]
        return False,None
    
    def calculate_path_legnth(self,points):
        length = 0
        for i in range(len(points.poses)-1):
            length += self.caluculate_distance(
                (points.poses[i].pose.position.x,points.poses[i].pose.position.y),
                (points.poses[i+1].pose.position.x,points.poses[i+1].pose.position.y)
                )
        return length

    def submit_path(self,target_id,path,path_type='initial'):
        #prepare payload
        payload = {
            'node_id':self.node_id,
            'target_id':target_id,
            'node_type': self.node_type,
            'path_type': path_type,
            'path_points':json.dumps(self.format_path(path)),
            'pos_x': self.targets[target_id]['pos_x'],
            'pos_y': self.targets[target_id]['pos_y'],
            'distance':self.calculate_path_legnth(path),
            'timecreated':datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        self.add_waiting_message(payload,'paths')
        self.submit_message(table_name='paths',message=json.dumps(payload))
        self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:publishing,path,{target_id}")
    def plan_path(self,target_id,avoid_conflicts= False):
        self.planner.setGoal(x=self.targets[target_id]['pos_x'],y=self.targets[target_id]['pos_y'],z=0)
        self.planner.plan()
        return self.planner.path
            
    def submit_node_state(self):
        #submit node state to blockchain
        loginfo('Task_allocator: submit node state')
        payload = {
            'node_id':self.node_id,
            'node_type':self.node_type,
            'timecreated':datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'pos_x':self.pos_x,
            'pos_y':self.pos_y,
            'details': ""
        }
        #self.add_waiting_message(payload,'states')
        #msg = SubmitTransactionRequest('states',json.dumps(payload))
        self.submit_message('states',json.dumps(payload))

    def loop(self):
        #update position
        self.update_position()
        #wait until chain is ready
        
        status = self.chain_status()
        
        if status.success == False or status.message != 'Ready':
            loginfo(f"{self.node_id}: Task_allocator: Waiting for chain to be ready, current status is {status.message}")
            return
      
        #check if time interval is reached since last state update
        if (datetime.now() - self.last_state).total_seconds() > self.update_interval:
            self.last_state = datetime.now()
            self.submit_node_state() 
            loginfo(f"{self.node_id}: Task_allocator: Node state submitted")
        #sync the robot with blockchain
        self.sync_records()
        #check if robot it idle
        if self.ongoing_task != None:
            self.check_ongoing_task()
            return
        
        #check if anything in waiting list
        if self.is_in_waiting():
            return
        
        is_committed,record = self.is_committed()
        if not is_committed:
            #get best target for me
            loginfo(f"{self.node_id}: Task_allocator: Checking for best target@@@")
            target_id = self.get_best_target()
            if target_id == None:
                loginfo(f"{self.node_id}: Task_allocator: from targets {self.targets} No target found@@@")
                return 
            loginfo(f"{self.node_id}: Task_allocator: Target found@@@")
            #commit to target
            self.commmit_to_target(target_id)
            return
        if not self.is_task_fully_committed(record['target_id']):
            return
        
        loginfo(f"{self.node_id}: Task_allocator: Task is fully committed@@@")
        is_submitted,path = self.is_path_submitted(record['target_id'])
        if not is_submitted:
            #plan a path for the target and submit it to waiting list
            if self.is_in_waiting(
                {'node_id':self.node_id,
                 'target_id':record['target_id']},
                 'paths'):
                return
            
            #plan a path for the target and submit it to waiting list
            path = self.plan_path(record['target_id'])
            if path == None:
                return
            #submit path to waiting list
            loginfo(f"{self.node_id}: {record} is commitrecord@@@")
            self.submit_path(record['target_id'],path)
            return
        loginfo(f"{self.node_id}: Task_allocator: Path is submitted@@@")
        
        #check if paths is all there 
        if not self.is_task_executable(record['target_id']):
            return
        
        loginfo(f"{self.node_id}: Task_allocator: Task is executable@@@")
        #check if there any conflicts
        #conflicted_ids = self.check_conflict(record['target_id'])
        conflicted_ids = []
        if len (conflicted_ids) == 0:
            #allocate robots to target
            self.log_publisher.publish(f"{self.node_id}:{mktime(datetime.now().timetuple())}:published,{self.node_id},started,{record['target_id']}")
            #self.visualize_path(path)
            self.start_task(path)
            self.ongoing_task = record['target_id']
            return
        
        loginfo(f"{self.node_id}: Task_allocator: Conflict detected@@@")
        
        #if found conflict, check if I need to plan path again
        is_conflicted = False
        for conf in conflicted_ids:
            if self.node_id in conf:
                is_conflicted = True
                break
        
        if is_conflicted:
            path = self.plan_path(record['target_id'],True)
            if path == None:
                return
            self.submit_path(record['target_id'],path,'replan')

        


if __name__ == "__main__":
    

    loginfo("task_allocator:Starting the task allocation node")
    robot = TaskAllocationManager(planningAlgorithm="AStar")
    while not is_shutdown():
        robot.loop()
