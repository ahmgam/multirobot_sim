
from time import mktime
import datetime
###############################################
# Queues managers
###############################################

class OrderedQueue:
    
    def __init__(self,base_dir="/"):
        self.base_dir = base_dir
        self.queue = []

    def put(self, message,msg_time,alt_key=None):
        
        #add message to queue
        self.queue.append({
            "message": message,
            "time": msg_time,
            "alt_key": alt_key
        })
     
        try:
            if alt_key:
                self.queue=sorted(self.queue,key=lambda x: (x['time'],x['alt_key']))
            else:
                self.queue=sorted(self.queue,key=lambda x: x['time'])
        except KeyError as e:
            print(self.queue)
            exit()
                 
    def pop(self):
        #get message from send queue
        if len(self.queue) == 0:
            return None
        else:
            data = self.queue.pop(0)
            return data
        
    def is_empty(self):
        return len(self.queue) == 0
    
    
    def count(self):
        return len(self.queue)
    
    def save(self):
        import json
        with open(self.base_dir+"/buffer.json", 'w') as f:
            json.dump(self.queue, f)
    
    def load(self):
        import json
        try:
            with open(self.base_dir+"/buffer.json", 'r') as f:
                self.queue = json.load(f)
        except:
            self.queue = []
    
