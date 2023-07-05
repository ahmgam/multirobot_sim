from multirobot_sim.srv import GetBCRecords,SubmitTransaction
from rospy import ServiceProxy
import json
from actionlib import SimpleActionClient
from rospy import ServiceProxy
import numpy as np
from multirobot_sim.action import NavigationAction


class TaskAllocationManager:
    def __init__(self,node_id):
        self.node_id = node_id
        self.robots = {}
        self.targets = {}
        self.tasks = {}
        self.records= {}
        self.paths = {}
        self.idle= {}
        self.waiting_message = None
        self.last_id = 0
        self.get_blockchain_records = ServiceProxy(f'{self.node_id}/get_records',GetBCRecords)
        self.submit_message = ServiceProxy(f'{self.node_id}/submit_message',SubmitTransaction)
        self.get_blockchain_records.wait_for_service()
        self.submit_message.wait_for_service()
        self.navigation_client = SimpleActionClient(f'{self.node_id}/navigation',NavigationAction)
        #self.get_blockchain_records = ServiceProxy('get_blockchain_records')

    def sync_records(self):
        #get new records from blockchain service 
        records = self.get_blockchain_records(self.last_id)
        for record in records:
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

    def update_state(self,state):
        pass

    def commmit_to_target(self,robot_id,target):
        pass

    def get_robot_state(self,robot_id):
        pass

    def allocate_robots_to_target(self,task_id):
        pass

    def send_to_blockchain(self,message):
        pass

    def set_initial_path_to_target(self,target):
        pass

    def reset_path_to_target(self,robot_id,paths):
        pass

    def check_conflict(self,paths):
        pass

    def is_committed(self):
        #check if current robot has committed to a target
        for task in self.tasks.values():
            for record in task:
                if record['node_id'] == self.node_id and record['record_type'] == 'commit':
                    return True,self.records[record['id']]
        return False,None
    
    
    def is_in_waiting(self,message=None):
        #check if message is in waiting
        if not message:
            return True if self.waiting else False
        else:
            if type(message) == str:
                message = json.loads(message)
            for key in list(set(message.keys()).intersection(set(self.waiting.keys()))):
                if self.waiting_message[key] != message[key]:
                    return False
            return True

    def check_ongoing_task(self):
        pass
    def loop(self):
        #sync the robot with blockchain
        self.sync_records()
        #check if robot it idle
        if not self.is_robot_idle(self.node_id):
            self.check_ongoing_task()
            return
        
if __name__ == "__main__":
    robot = TaskAllocationManager("s")
    robot.loop()