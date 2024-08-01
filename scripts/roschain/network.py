#!/usr/bin/env python
from messages import *
import datetime
from queue import Queue
from encryption import EncryptionModule
from rospy import loginfo
from time import mktime
from rospy import Subscriber,Publisher,ROSInterruptException,Service,ServiceProxy,Rate,init_node,get_namespace,get_param,is_shutdown
from multirobot_sim.srv import FunctionCall,FunctionCallResponse
from std_msgs.msg import String
from time import time
import pickle
from messages import MessagePublisher,MessageSubscriber
class NetworkInterface:
    
    def __init__(self,node_id,node_type,DEBUG=True):
        '''
        Initialize network interface
        '''
        #define node id
        self.node_id = node_id
        #define node type
        self.node_type = node_type
        #define debug mode
        self.DEBUG = DEBUG
        #define discovery interval
        self.discovery_interval = 10
        #define heartbeat interval
        self.heartbeat_interval = 5
        #init node
        self.node = init_node("network_interface", anonymous=True)
        #define queue
        self.queue = Queue()
        #define retry queue
        self.retry_queue = Queue()
        #define key store proxy
        loginfo(f"{self.node_id}: NetworkInterface:Initializing key store service")
        self.key_store = ServiceProxy(f"/{self.node_id}/key_store/call", FunctionCall)
        self.key_store.wait_for_service(timeout=100)
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk,self.sk =EncryptionModule.reconstruct_keys(keys["pk"],keys["sk"])
        #define sessions service proxy
        loginfo(f"{self.node_id}: NetworkInterface:Initializing sessions service")
        self.sessions = ServiceProxy(f"/{self.node_id}/sessions/call", FunctionCall,True)
        self.sessions.wait_for_service(timeout=100)
        self.session_cache = self.make_function_call(self.sessions,"get_connection_sessions")
        #define connector subscriber
        loginfo(f"{self.node_id}: NetworkInterface:Initializing connector subscriber")
        self.subscriber = MessageSubscriber(f"/{self.node_id}/network/handle_message", self.to_queue,("handle",))
        #define network prepaeration service
        self.prepare_subscriber = MessageSubscriber(f"/{self.node_id}/network/prepare_message", self.to_queue,("prepare",))
        #define connector publisher
        self.publisher = MessagePublisher(f"/{self.node_id}/connector/send_message")
        #define discovery publisher
        self.discovery_publisher = MessagePublisher(f"/{self.node_id}/discovery/discovery_handler")
        #define heartbeat publisher
        self.heartbeat_publisher = MessagePublisher(f"/{self.node_id}/heartbeat/heartbeat_handler")
        # Define consensus publisher
        self.consensus_publisher = MessagePublisher(f"/{self.node_id}/consensus/consensus_handler")
        #Define sync publisher
        self.sync_publisher = MessagePublisher(f"/{self.node_id}/blockchain/sync_handler")
        #define server
        loginfo(f"{self.node_id}: NetworkInterface:Initializing network service")
        self.server = Service(f"/{self.node_id}/network/call", FunctionCall, self.handle_function_call)
        #define is_initialized
        loginfo(f"{self.node_id}: NetworkInterface:Initialized successfully")
        
    def handle_function_call(self,req):
        #get function name and arguments from request
        function_name = req.function_name
        args = json.loads(req.args)
        if type(args) is not list:
            args = [args]
        #call function
        if hasattr(self,function_name):
            if len(args) == 0:
                response = getattr(self,function_name)()
            else:
                response = getattr(self,function_name)(*args)
        else:
            response = None
        if response is None:
            response = FunctionCallResponse(r'{}')
        else:
            response = json.dumps(response) if type(response) is not str else response
            response = FunctionCallResponse(response)
        return response
    
    def to_queue(self,message,type):
        '''
        Add message to queue
        '''        
        self.queue.put({"type":type[0],"data":message})
     
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args)
        
        if response == r"{}":
            ret_data= None
        else:
            ret_data = json.loads(response.response)
        return ret_data
    def verify_data(self,message):
        #check if message has session id
        original_message = message
        message = message["data"]
        if message.get("session_id",'') == "": 
            #the message has no session id, so it's discovery message
            #verify the message hash 
            buff = message
            msg_signature = buff.get('signature')
            if msg_signature != None:
                del buff['signature']
            #check if message is string
            if type(message["message"]) is  str:
                #the message is a string, so it's encrypted discovery message
                #check if the node does not have active discovery session with the sender
                
                try:
                    decrypted_data = EncryptionModule.decrypt(message["message"],self.sk)
                    #parse the message
                    if type(decrypted_data) is not dict:
                        if self.DEBUG:
                            loginfo(f"{self.node_id}: Invalid message data {decrypted_data}")
                        
                except :
                    if self.DEBUG:    
                        loginfo(f"{self.node_id}: error decrypting and parsing data : {e}")
                    return None
                #validate the message
                message["message"] = decrypted_data
                buff["message"] = decrypted_data
                if decrypted_data["data"].get("discovery_session_id"):
                    session = self.make_function_call(self.sessions,"get_discovery_session",decrypted_data["data"]["discovery_session_id"])
                else:
                    session = self.make_function_call(self.sessions,"get_discovery_session_by_node_id",message["node_id"])
                if not session:
                    try:
                        session = {"pk": decrypted_data["data"]["pk"]}
                    except Exception as e:
                        raise Exception(f"error with message of type {message['type']} and session id {message['session_id']}")
                
            else:
                #the message is not a string, so it's not encrypted discovery message
                try:
                    session = {"pk":message["message"]["data"]["pk"]}
                except Exception as e:
                    raise Exception(f"error with message of type {message['type']}")
                
            #verify the message signature
            if msg_signature:
                if EncryptionModule.verify(buff, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
                    if self.DEBUG:    
                        loginfo(f"{self.node_id}: signature not verified")
                    return None
            return {
                "message": message,
                "session": session
            }
        else:
            #get session
            #session = self.session_cache.get(message["session_id"])
            session = self.make_function_call(self.sessions,"get_connection_session",message["session_id"])
            if not session:
                dis_session = self.make_function_call(self.sessions,"get_discovery_session",message["session_id"])
                if dis_session:
                    loginfo(f"{self.node_id}: session not found in connection sessions, but it's still in discovery sessions")
                    #return the message to the queue
                    self.retry_queue.put(original_message)
                    return None
                
                if self.DEBUG:
                    session_by_id = self.make_function_call(self.sessions,"get_connection_sessions")
                    loginfo(f"{self.node_id}: Invalid session in network message, with node {message['node_id']}, message type : {message['type']}, session : {message['session_id']} , sessions : {session_by_id}")
                return

            #decrypt message
            try:
                decrypted_data = EncryptionModule.decrypt_symmetric(message["message"],session["key"])
                if type(decrypted_data) is not dict:
                    loginfo(f"{self.node_id}: Invalid message data {decrypted_data}")
            except Exception as e:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: error in symmetric decryption : {e}")
                return
            #validate message
            message["message"] = decrypted_data       
            return {
                "message": message,
                "session": session
            }
    
    def __prepare_message(self,msg_type, message,signed=False,session_id=None):
        
        #prepare message payload
        msg_payload = OrderedDict({
            "type": msg_type,
            "time":mktime(datetime.datetime.now().timetuple()),
            "node_id": self.node_id,
            "node_type": self.node_type,
            "session_id": session_id if session_id else '',
            "message": message
            })
        #check if signed 
        if signed:
            msg_payload["signature"] = EncryptionModule.sign(msg_payload,self.sk)
        return msg_payload
    

    def send_message(self,msg_type, target, message,signed=False):
        #prepare message payload
        msg_data = {
            "timestamp": str(datetime.datetime.now()),
                "data":message
                }
        
        msg_payload = self.__prepare_message(msg_type,msg_data,signed=signed)
        #define target sessions
        if target == "all":
            #prepare message data
            self.publisher.publish({
                    "target": 'all',
                    "time":mktime(datetime.datetime.now().timetuple()),
                    "message": msg_payload
                })
        else:
            sessions = self.make_function_call(self.sessions,"get_connection_sessions")
            sessions = {session["node_id"]:session for session in sessions.values()}
            if target == "all_active":
                pass
            elif type(target) == list:
                sessions = {node_id:sessions.get(node_id) for node_id in target}
            else:
                sessions = {target:sessions.get(target)}
            #iterate over target sessions
            for node_id,session in sessions.items():
                #check if node_id is local node_id 
                if node_id == self.node_id:
                    continue
                if session != None and session["approved"]:
                    #encrypt message data
                    prepared_message = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
                    msg_payload["message"] = prepared_message
                    msg_payload["session_id"] = session["session_id"]
                else:
                    #check if message payload contains discovery session id
                    if message.get("discovery_session_id"):
                        session = self.make_function_call(self.sessions,"get_discovery_session",message["discovery_session_id"])
                    else:
                        #check if there is discovery session
                        session = self.make_function_call(self.sessions,"get_discovery_session_by_node_id",node_id)
                    if session:
                        #encrypt message data
                        prepared_message = EncryptionModule.encrypt(msg_data,EncryptionModule.reformat_public_key(session["pk"]))
                        msg_payload["message"] = prepared_message
                    
                #add message to the queue
                self.publisher.publish({
                    "target": node_id,
                    "time":mktime(datetime.datetime.now().timetuple()),
                    "message": msg_payload
                })
  
            
    def handle_message(self, message):
        #check message type
        #print(message)
        verified =self.verify_data(message)
        if not verified:
            return
        message= verified["message"]
        session = verified["session"]
        #handle message                      
        if message["node_id"]==self.node_id:
            return
        elif str(message["type"]).startswith("discovery"):
            self.discovery_publisher.publish(message)
        elif str(message["type"]).startswith("heartbeat"):
            self.heartbeat_publisher.publish(message)
        elif str(message["type"]).startswith("sync"):
            self.sync_publisher.publish(message)
        elif message["type"]=="data_exchange":
            self.consensus_publisher.publish({"message":message["message"],"session":session})
        else:
            if self.DEBUG:
                loginfo(f"{self.node_id}: unknown message type {message['type']}")
        
    
if __name__ == "__main__":
    ns = get_namespace()
    try :
        node_id= get_param(f'{ns}roschain/node_id') # node_name/argsname
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")
    
    try:
        node_type= get_param(f'{ns}roschain/node_type') # node_name/argsname
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_type")
    
    network = NetworkInterface(node_id,node_type)
    rate = Rate(10)
    counter = 0
    while not is_shutdown():
        if not network.queue.empty():
            message = network.queue.get()
            #loginfo(f"{network.node_id}: Network: Handling message of type {message['data']['type']}")
            #start_time = time()
            if message["type"] == "handle":
                #print(message["data"])
                network.handle_message(message)
                #print(f"Time taken to handle message : {time()-start_time}")
            elif message["type"] == "prepare":
                network.send_message(message["data"]["type"],message["data"]["target"],message["data"]["message"],message["data"].get("signed",False))
                #print(f"Time taken to send message : {time()-start_time}")            
            else:
                loginfo(f"{network.node_id}: Invalid message type on network node, message : {message}")
        if not network.retry_queue.empty():
            message = network.retry_queue.get()
            network.handle_message(message)
        if counter == 10:
            counter = 0
            network.session_cache = network.make_function_call(network.sessions,"get_connection_sessions")
        counter +=1
        rate.sleep()
   