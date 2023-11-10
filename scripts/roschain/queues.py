
from time import mktime
import datetime
###############################################
# Queues managers
###############################################

class QueueManager:
    
    def __init__(self):
        self.queue = []
        self.output_queue = []
        self.consensus_queue = []
        
    def put_queue(self, message,msg_type,on_failure=None):
        
        #add message to queue
        self.queue.append({
            "message": message,
            "time": mktime(datetime.datetime.now().timetuple()) if message.get("time",None) is None else message["time"],
            "type": msg_type,
            "on_failure": on_failure
        })
     
        try:
            self.queue.sort(key=lambda x: x['time'])
        except KeyError as e:
            print(self.queue)
            exit()
                 
    def pop_queue(self):
        #get message from send queue
        if len(self.queue) == 0:
            return None
        else:
            data = self.queue.pop(0)
            return data
        
    @property
    def queue_empty(self):
        return len(self.queue) == 0
    
    @property
    def queue_count(self):
        return len(self.queue)
    
    def put_output_queue(self, message,msg_source,msg_type,timestamp=mktime(datetime.datetime.now().timetuple())):
        
        #add message to queue
        self.output_queue.append({
            "message": message,
            "source": msg_source,
            "type": msg_type,
            "time": timestamp
        })
        self.output_queue.sort(key=lambda x: x['time'])
    
                 
    def pop_output_queue(self):
        #get message from send queue
        if len(self.output_queue)==0:
            return None
        else:
            data = self.output_queue.pop(0)
            return data    
        
    @property
    def output_queue_empty(self):
        return len(self.output_queue) == 0
    
    @property
    def output_queue_count(self):
        return len(self.output_queue)
    
    def put_consensus_queue(self, message,msg_source,msg_type,timestamp=mktime(datetime.datetime.now().timetuple())):
        
        #add message to queue
        self.consensus_queue.append({
            "message": message,
            "source": msg_source,
            "type": msg_type,
            "time": timestamp
        })
        self.consensus_queue.sort(key=lambda x: x['time'])
    
                 
    def pop_consensus_queue(self):
        #get message from send queue
        if len(self.consensus_queue)==0:
            return None
        else:
            data = self.consensus_queue.pop(0)
            return data    
        
    @property
    def consensus_queue_empty(self):
        return len(self.consensus_queue) == 0
    
    @property
    def consensus_queue_count(self):
        return len(self.consensus_queue)
