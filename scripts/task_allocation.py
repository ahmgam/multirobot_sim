from multirobot_sim.srv import GetBCRecords,SubmitTransaction
from rospy import ServiceProxy
import json
from actionlib import SimpleActionClient,GoalStatus
from rospy import ServiceProxy
import numpy as np
from multirobot_sim.action import NavigationAction,NavigationActionGoal
from geometry_msgs.msg import Point


class TaskAllocationManager:
    def __init__(self,node_id):
        self.node_id = node_id
        self.robots = {}
        self.targets = {}
        self.tasks = {}
        self.records= {}
        self.paths = {}
        self.idle= {self.node_id:True}
        self.waiting_message = None
        self.ongoing_task = None
        self.last_id = 0
        self.get_blockchain_records = ServiceProxy(f'{self.node_id}/get_records',GetBCRecords)
        self.submit_message = ServiceProxy(f'{self.node_id}/submit_message',SubmitTransaction)
        self.get_blockchain_records.wait_for_service()
        self.submit_message.wait_for_service()
        self.navigation_client = SimpleActionClient(f'{self.node_id}/navigation',NavigationAction)
        #self.get_blockchain_records = ServiceProxy('get_blockchain_records')

    def sync_records(self):
        #get new records from blockchain service 
        records = self.get_blockchain_records(GetBCRecords(last_trans_id=self.last_id))
        for record in records:
            record = json.loads(record)
            self.process_record(record)
            
    def process_record(self,record):
        if record['meta']['table'] == 'states':
            self.robots[record['data']['node_id']] = record['data']
        if record['meta']['table'] == 'targets':
            self.targets[record['data']['node_id']] = record['data']
        if record['meta']['table'] == 'task_records':
            data = record['data']
            if record['record_type'] == 'commit':
                self.idle[data['node_id']] = False
            else:
                self.idle[data['node_id']] = True
            if data['target_id'] in self.tasks.keys():
                self.tasks[data['targetid']].append(data)
            else:
                self.tasks[data['target_id']] = [data]
            self.records[data['id']] = data
            if self.is_task_completed(data['target_id']):
                self.clear_task(data['target_id'])

        if record['meta']['table'] == 'path':
            data = record['data']
            target_id = self.records[data['commit_id']]['target_id']
            if target_id in self.paths.keys():
                self.paths[target_id][data['commit_id']] = data
            else:
                self.paths[target_id] = {}
                self.records[target_id][data['commit_id']] = data
        self.last_id = record['meta']['id']
        if self.is_in_waiting(record['data'],record['meta']['table']):
                self.waiting_message = None

    def is_task_fully_committed(self,task_id):
        #check all records in task
        task = self.tasks[task_id]
        req_uav = int(self.targets[task[0]['target_id']]['needed_uav'])
        req_ugv = int(self.targets[task[0]['target_id']]['needed_ugv'])
        committed_uav = 0
        committed_ugv = 0
        for record in task:
            if record['record_type'] == 'commit':
                if record['node_type'] == 'uav':
                    committed_uav += 1
                if record['node_type'] == 'ugv':
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
        for commit_id,path in paths.items():
            if path['node_type'] == 'uav':
                planned_uav[commit_id] = path
            if path['node_type'] == 'ugv':
                planned_ugv[commit_id] = path

        if len(planned_uav.keys()) != req_uav or len(planned_ugv.keys()) != req_ugv:
            return False
        #check paths conflicts 
        return self.check_conflict([p["path_points"] for p in paths.values()]) and self.check_conflict([planned_uav[p]["path_points"] for p in planned_uav.keys()])
            
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
        task = self.tasks[task_id]
        #get all commit ids 
        record_ids = task.keys()
        for record_id in record_ids:
            target_id = task[record_id]['target_id']
            if target_id in self.targets.keys():
                del self.targets[target_id]
            del self.records[record_id]
        del self.tasks[task_id]
        del self.paths[task_id]

    def get_target_best_candidates(self,target_id):
        #get task details 
        goal = self.targets[target_id]['pos_x'],self.targets[target_id]['pos_y']
        for robot in self.robots.values():
            distances = []
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
                    best_targets.append(robot)

        #sort best targets
        best_targets.sort(key=lambda x: x['distance'], reverse=True)
        if len(best_targets) > 0:
            target_id = best_targets[0]['node_id']
        return target_id

    def is_task_executable(self,target_id):
        #get needed uav and ugv
        needed_uav = int(self.targets[target_id]['needed_uav'])
        needed_ugv = int(self.targets[target_id]['needed_ugv'])
        #get all idle robots
        for robot in self.robots.values():
            if self.is_robot_idle(robot['node_id']):
                if robot['node_type'] == 'uav':
                    needed_uav -= 1
                if robot['node_type'] == 'ugv':
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
    
    def add_target(self,target):
        pass

    def commmit_to_target(self,target):
        #prepare payload
        payload = {
            'node_id':self.node_id,
            'target_id':target,
            'record_type':'commit'
        }
        self.add_waiting_message(payload,'task_records')
        msg = SubmitTransaction(table_name='task_records',message=json.dumps(payload))
        self.submit_message(msg)

    def add_waiting_message(self,message,msg_type):
        self.waiting_message = {
            "type":msg_type,
            "message":message
        }
    
    def format_path(self,path):
        #convert path from list of tuples to list of points
        points = []
        for point in path:
            points.append(Point(point[0],point[1],0))
        return points
    
    def start_task(self,path):
        goal = NavigationActionGoal(points=self.format_path(path))
        self.navigation_client.send_goal(goal)
        
    def check_conflict(self,target_id):
        all_paths = self.paths[target_id].values()
        conflicted_paths = []
        while len(all_paths) > 0:
            first_path = all_paths[0]
            for i in range(1,len(all_paths)):
                if self.are_paths_intersection(first_path["path_points"],all_paths[i]["path_points"]):
                    conflicted_paths.append((first_path["commit_id"],all_paths[i]["commit_id"]))
                    
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
                    return True,self.records[record['id']]
        return False,None
    
    
    def is_in_waiting(self,message=None,msg_type=None):
        if self.waiting_message == None:
            return False
        #check if message is in waiting
        if message!= None and msg_type != None:
            if type(message) == str:
                message = json.loads(message)
            if msg_type != self.waiting_message['type']:
                return False
            for key in list(set(message.keys()).intersection(set(self.waiting_message['message'].keys()))):
                if self.waiting_message['message'][key] != message[key]:
                    return False
            return True
        return True
    def send_complete_message(self):
        #send complete message to blockchain
        payload = {
            'node_id':self.node_id,
            'record_type':'complete',
            'target_id':self.ongoing_task['target_id']
        }
        self.add_waiting_message(payload,'task_records')
        msg = SubmitTransaction(table_name='task_records',message=json.dumps(payload))
        
    def check_ongoing_task(self):
        #check the status of ongoing task
        if self.navigation_client.get_state() == GoalStatus.SUCCEEDED:
            #set robot state to idle
            self.idle[self.node_id] = True
            #send complete message to blockchain
            self.send_complete_message()
        return

    def is_path_submitted(self,target_id):
        for path in self.paths[target_id].values():
            if path['node_type'] == 'uav':
                return True,path
        return False,None
    
    def calculate_path_legnth(self,points):
        length = 0
        for i in range(len(points)-1):
            length += self.caluculate_distance(points[i],points[i+1])
        return length

    def submit_path(self,target_id,commit_id,path,path_type='initial'):
        #prepare payload
        payload = {
            'node_id':self.node_id,
            'target_id':target_id,
            'path_type':path_type,
            'node_type': self.node_type,
            'commit_id':commit_id,
            'path_points':json.dumps(path),
            'x_pos': self.targets[target_id]['pos_x'],
            'y_pos': self.targets[target_id]['pos_y'],
            'distance':self.calculate_path_legnth(path)
        }
        self.add_waiting_message(payload,'paths')
        msg = SubmitTransaction(table_name='paths',message=json.dumps(payload))
        self.submit_message(msg)
    def plan_path(self,target_id,avoid_conflicts= False):
        pass
            
    def loop(self):
        #sync the robot with blockchain
        self.sync_records()
        #check if robot it idle
        if not self.is_robot_idle(self.node_id):
            self.check_ongoing_task()
            return
        
        #check if anything in waiting list
        if self.is_in_waiting():
            return
        
        is_committed,record = self.is_committed()
        if not is_committed:
            #get best target for me
            target_id = self.get_best_target()
            if target_id == None:
                return 
            #commit to target
            self.commmit_to_target(target_id)
            return
        if not self.is_task_fully_committed(record['target_id']):
            return
        
        is_submitted,path = self.is_path_submitted(record['target_id'])
        if not is_submitted:
            #plan a path for the target and submit it to waiting list
            if self.is_in_waiting(
                {'node_id':self.node_id,
                 'target_id':record['target_id']},
                 'path'):
                return
            
            #plan a path for the target and submit it to waiting list
            path = self.plan_path(record['target_id'])
            if path == None:
                return
            #submit path to waiting list
            self.submit_path(record['target_id'],record['id'],path)
            return
        
        #check if paths is all there 
        if not self.is_task_executable(record['target_id']):
            return
        
        #check if there any conflicts
        conflicted_ids = self.check_conflict(record['target_id'])

        if len (conflicted_ids) == 0:
            #allocate robots to target
            self.start_task(path)
            return
        
        #if found conflict, check if I need to plan path again
        is_conflicted = False
        for conf in conflicted_ids:
            if record["id"] in conf:
                is_conflicted = True
                break
        if is_conflicted:
            path = self.plan_path(record['target_id'],True)
            if path == None:
                return
            self.submit_path(record['target_id'],record['id'],path,'reset')

            
        
        
if __name__ == "__main__":
    robot = TaskAllocationManager("s")
    robot.loop()