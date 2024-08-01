#!/usr/bin/env python
import json
import datetime
from rospy import init_node,get_param,loginfo,get_namespace,spin,ROSInterruptException,Service,ServiceProxy,Publisher
from time import mktime
from multirobot_sim.srv import GetBCRecords,SubmitTransaction,GetBCRecordsResponse,SubmitTransactionResponse,FunctionCall
from std_srvs.srv import Trigger,TriggerResponse
from std_msgs.msg import String
from random import choices,randint
from string import ascii_lowercase
from messages import MessagePublisher,MessageSubscriber
#from multirobot_sim.srv import GetBCRecords,SubmitTransaction
#####################################
# RosChain Module
#####################################

class RosChain:
    def __init__(self,node_id,node_type,min_nodes_num,DEBUG=False):
        '''
        Initialize network interface
        '''
        #define is_initialized
        self.ready = False
        #define debug mode
        self.DEBUG = DEBUG
        #define node id
        self.node_id = node_id
        #define node type
        self.node_type = node_type
        #define min nodes num
        self.min_nodes_num = min_nodes_num
        #define ros node
        self.node = init_node("roschain", anonymous=True)
        #define eady flag
        self.ready = False
        
        #define records service
        loginfo(f"{self.node_id}: ROSChain:Initializing records service")
        self.get_record_service = Service(f"/{self.node_id}/roschain/get_records",GetBCRecords,lambda req: self.get_records(req))
        #define submit message service
        loginfo(f"{self.node_id}: ROSChain:Initializing submit message service")
        self.submit_message_service = Service(f"/{self.node_id}/roschain/submit_message",SubmitTransaction,self.submit_message)
        #init sessions
        loginfo(f"{self.node_id}: SBFT:Initializing sessions service")
        self.sessions = ServiceProxy(f"/{self.node_id}/sessions/call",FunctionCall,True)
        self.sessions.wait_for_service(timeout=100)
        #define blockchain service proxy 
        loginfo(f"{self.node_id}: ROSChain:Initializing blockchain service")
        self.blockchain = ServiceProxy(f"/{self.node_id}/blockchain/call", FunctionCall)
        self.blockchain.wait_for_service(timeout=100)
        #define consensus service
        loginfo(f"{self.node_id}: ROSChain:Initializing consensus service")
        self.consensus = MessagePublisher(f"/{self.node_id}/consensus/consensus_handler")
        loginfo(f"{self.node_id}: RSOChain:Initialized successfully")
        #initialize ready service
        self.is_ready_service = Service(f"/{self.node_id}/roschain/is_ready",Trigger,self.is_ready)
        #define connector log publisher
        self.log_publisher = Publisher(f"/{self.node_id}/connector/send_log", String, queue_size=10)
        self.counter=0
        self.min_num_connected_time = None
        self.ready = True
        
    def is_ready(self,msg):
        is_connected =len(self.make_function_call(self.sessions,"get_active_nodes")) >= self.min_nodes_num
        if is_connected and  not self.min_num_connected_time:
            self.min_num_connected_time = mktime(datetime.datetime.now().timetuple())
        if self.min_num_connected_time:
            time_passed = mktime(datetime.datetime.now().timetuple()) - self.min_num_connected_time >= 10
        else:
            time_passed = False
        if is_connected and self.ready and time_passed:
            self.ready = True
            return TriggerResponse(True,"Ready")
        else:
            return TriggerResponse(False,"Not Ready")
        
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    
    def submit_message(self,args):
        '''
        Send message to the given public key
        '''
        loginfo(f"{self.node_id}: ROSChain: {self.node_id} is sending message of type {args.table_name}")
        table_name = args.table_name
        data =json.loads(args.message) 
        msg_time = mktime(datetime.datetime.now().timetuple())
        msg_id = f'{self.node_id}_{self.counter}'
        message = {
            "table_name":table_name,
            "data":data,
            "time":msg_time,
            "msg_id":msg_id
            #"time":datetime.datetime.fromtimestamp(msg_time).strftime("%Y-%m-%d %H:%M:%S") 
        }
        #log_msg = f"{msg_time},msg,{msg_id}"
        #self.log_publisher.publish(log_msg)
        #payload 
        payload ={
            "message":message,
            "source":self.node_id,
            "timestamp":msg_time,
            "operation": "submit"
        }
        msg = {"message":{"data": payload}}
        #add message to the parent queue
        self.counter+=1
        self.consensus.publish(msg)
        return SubmitTransactionResponse("Success")

    def get_records(self,last_record):
        records = []
        try:
            for id in range(last_record.last_trans_id,int(self.make_function_call(self.blockchain,"get_last_id","blockchain"))+1):
                meta,data = self.make_function_call(self.blockchain,"get_transaction",id)
                records.append(json.dumps({
                    f"{id}":{"meta":meta,"data":data}
                }))
        except:
            records = []
        return GetBCRecordsResponse(records)

#####################################
# Main
#####################################             

if __name__ == "__main__":         
    ns = get_namespace()
    try :
        node_id= get_param(f'{ns}roschain/node_id') # node_name/argsname
        loginfo("ROSCHAIN: Getting node_id argument, and got : ", node_id)

    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")

    try :
        node_type= get_param(f'{ns}roschain/node_type') # node_name/argsname
        loginfo("ROSCHAIN: Getting node_type argument, and got : ", node_type)

    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_type")
    try :
        min_nodes_num= get_param(f'{ns}roschain/min_nodes_num',1) # node_name/argsname
        loginfo(f"discovery: Getting min_nodes_num argument, and got : {min_nodes_num}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : min_nodes_num")
    node = RosChain(node_id,node_type,min_nodes_num,True)
  
    spin()
    
            