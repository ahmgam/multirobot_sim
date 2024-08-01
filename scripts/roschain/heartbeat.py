#!/usr/bin/env python
from collections import OrderedDict
from encryption import EncryptionModule
from queue import Queue
import datetime
import json
from time import mktime
from rospy import loginfo,ServiceProxy,init_node,Publisher,get_namespace,spin,get_param,is_shutdown,Rate,ROSInterruptException,Subscriber
from std_msgs.msg import String
from multirobot_sim.srv import FunctionCall
from random import randint
from messages import MessagePublisher,MessageSubscriber
class HeartbeatProtocol:
    
    def __init__(self,node_id,node_type,max_delay,DEBUG=True):
        self.node_id = node_id
        self.node_type = node_type
        self.DEBUG = DEBUG
        #define heartbeat interval
        self.heartbeat_interval = 5
        #define node
        self.node = init_node("heartbeat_protocol", anonymous=True)
        #define key store proxy
        loginfo(f"{self.node_id}: HeartbeatProtocol:Initializing key store service")
        self.key_store = ServiceProxy(f"/{self.node_id}/key_store/call", FunctionCall)
        self.key_store.wait_for_service(timeout=100)
        #define sessions
        loginfo(f"{self.node_id}: HeartbeatProtocol:Initializing sessions service")
        self.sessions = ServiceProxy(f"/{self.node_id}/sessions/call", FunctionCall,True)
        self.sessions.wait_for_service(timeout=100)
        #define blockchain
        loginfo(f"{self.node_id}: HeartbeatProtocol:Initializing blockchain service")
        self.blockchain = ServiceProxy(f"/{self.node_id}/blockchain/call", FunctionCall,True)
        self.blockchain.wait_for_service(timeout=100)
        #define heartbeat subscriber
        loginfo(f"{self.node_id}: HeartbeatProtocol:Initializing heartbeat subscriber")
        self.subscriber = MessageSubscriber(f"/{self.node_id}/heartbeat/heartbeat_handler", self.to_queue)
        #define network 
        self.prepare_message = MessagePublisher(f"/{self.node_id}/network/prepare_message")
        #define last heartbeat
        self.last_call = mktime(datetime.datetime.now().timetuple())+ randint(1,max_delay)
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk,self.sk =EncryptionModule.reconstruct_keys(keys["pk"],keys["sk"])
        #define queue
        self.queue = Queue()
        loginfo(f"{self.node_id}: HeartbeatProtocol:Initialized successfully")
           
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    
    def to_queue(self,data):
        self.queue.put(data)
    
    def cron(self):
        #send heartbeat to all nodes
        sessions = self.make_function_call(self.sessions,"get_connection_sessions")
        if sessions:
            for session_id, session in sessions.items():
                #check if time interval is passed
                session_time = mktime(datetime.datetime.now().timetuple()) - session["last_heartbeat"]
                if session_time > self.heartbeat_interval and session["status"] == "active":
                    #send heartbeat
                    self.send_heartbeat(session)
                    #update last heartbeat time
                    session["last_heartbeat"] = mktime(datetime.datetime.now().timetuple())
                    self.make_function_call(self.sessions,"update_connection_session",session_id,session)
    def handle(self,message):
        
        if message["type"] == "heartbeat_request":
            if self.DEBUG:
                loginfo(f"{self.node_id}: HeartbeatProtocol: Received message from {message['node_id']} of type {message['type']}, starting handle_heartbeat")
            self.handle_heartbeat(message)
        elif message["type"] == "heartbeat_response":
            if self.DEBUG:
                loginfo(f"{self.node_id}: HeartbeatProtocol: Received message from {message['node_id']} of type {message['type']}, starting handle_heartbeat_response")
            self.handle_heartbeat_response(message)
        else:
            if self.DEBUG:
                loginfo(f"{self.node_id}: HeartbeatProtocol: unknown message type {message['type']}")
                
    def send_heartbeat(self,session):
        
        #send heartbeat to session
        #prepare message 
        msg_data = {
                "blockchain_status":self.make_function_call(self.blockchain,"get_sync_info"),
                "state_table":self.make_function_call(self.sessions,"get_node_state_table")
            }
        #call network service
        loginfo(f"{self.node_id}: HeartbeatProtocol: Sending heartbeat to {session['node_id']}")
        self.prepare_message.publish({"message":msg_data,"type":"heartbeat_request","target":session["node_id"]})
        
           
    def handle_heartbeat(self,message):
        #receive heartbeat from node
        #get session
        session = self.make_function_call(self.sessions,"get_connection_session",message["session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid session in handle_heartbeat")
            return
        #check counter
        #if message["message"]["counter"]<=session["counter"]:
        #    if self.DEBUG:
        #        loginfo(f"{self.node_id}: Invalid counter")
        #    return
        #update node state table
        #self.parent.server.logger.warning(f'table request : {json.dumps(message["message"]["data"])}' )
        self.make_function_call(self.sessions,"update_node_state_table",message["message"]["data"]["state_table"])
        #update session
        self.make_function_call(self.sessions,"update_connection_session",message["session_id"],{
            "last_active": mktime(datetime.datetime.now().timetuple())})
        #chcek blockchain status
        is_synced = self.make_function_call(self.blockchain,"check_sync",message["message"]["data"]["blockchain_status"]["last_record"],message["message"]["data"]["blockchain_status"]["number_of_records"])
        if is_synced == "False":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Un synced blockchain, sending sync request")
            self.make_function_call(self.blockchain,"send_sync_request")
            
        #prepare message 
        msg_data = {
                "state_table":self.make_function_call(self.sessions,"get_node_state_table"),
                "blockchain_status":self.make_function_call(self.blockchain,"get_sync_info")
            }
        #call network service
        self.prepare_message.publish({"message":msg_data,"type":"heartbeat_response","target":session["node_id"]})
 
    def handle_heartbeat_response(self,message):
        #receive heartbeat from node
        #get session
        session = self.make_function_call(self.sessions,"get_connection_session",message["session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid session in handle_heartbeat_response")
            return
        #self.parent.server.logger.warning(f'table response : {json.dumps(message["message"]["data"])}' )
        #check counter
        #if message["message"]["counter"]<=session["counter"]:
        #    if self.DEBUG:
        #        loginfo(f"{self.node_id}: Invalid counter")
        #    return
        #update node state table
        self.make_function_call(self.sessions,"update_node_state_table",message["message"]["data"]["state_table"])
        #update session
        self.make_function_call(self.sessions,"update_connection_session",message["session_id"],{
            "last_active": mktime(datetime.datetime.now().timetuple())})
        #chcek blockchain status
        if self.make_function_call(self.blockchain,"check_sync",message["message"]["data"]["blockchain_status"]["last_record"],message["message"]["data"]["blockchain_status"]["number_of_records"])== False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Un synced blockchain, sending sync request")
            self.make_function_call(self.blockchain,"send_sync_request")    
            
if __name__ == '__main__':
    ns = get_namespace()
    
    try :
        node_id= get_param(f'{ns}heartbeat/node_id') # node_name/argsname
        loginfo(f"discovery: Getting node_id argument, and got : {node_id}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")
    
    try :
        node_type= get_param(f'{ns}heartbeat/node_type') # node_name/argsname
        loginfo(f"discovery: Getting endpoint argument, and got : {node_type}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_type")
    
    try:
        max_delay = get_param(f'{ns}heartbeat/max_delay',10)
        loginfo(f"discovery: Getting max_delay argument, and got : {max_delay}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : max_delay")
    
    node = HeartbeatProtocol(node_id,node_type,max_delay,DEBUG=False)
    rate = Rate(10)
    while not is_shutdown():
        #check queue
        node.cron()
        if not node.queue.empty():
            msg = node.queue.get()
            node.handle(msg)
        rate.sleep()