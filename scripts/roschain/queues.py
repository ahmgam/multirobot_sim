
from time import mktime
import datetime
###############################################
# Queues managers
###############################################

class OrderedQueue:
    
    def __init__(self):
        self.queue = []

    def put(self, message,msg_type):
        
        #add message to queue
        self.queue.append({
            "message": message,
            "time": mktime(datetime.datetime.now().timetuple()) if message.get("time",None) is None else message["time"],
            "type": msg_type
        })
     
        try:
            self.queue.sort(key=lambda x: x['time'])
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
    
