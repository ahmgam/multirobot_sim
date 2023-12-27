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
        #define server
        loginfo(f"{self.node_id}: NetworkInterface:Initializing network service")
        self.server = Service('call', FunctionCall, self.handle_function_call)
        #define queue
        self.queue = Queue()
        #define connector subscriber
        loginfo(f"{self.node_id}: NetworkInterface:Initializing connector subscriber")
        self.subscriber = Subscriber('handle_message', String, self.to_queue,("handle",))
        #define network prepaeration service
        self.prepare_subscriber = Subscriber('prepare_message', String, self.to_queue,("prepare",))
        #define connector publisher
        self.publisher = Publisher('send_message', String, queue_size=10)
        #define discovery publisher
        self.discovery_publisher = Publisher('discovery_handler', String, queue_size=10)
        #define heartbeat publisher
        self.heartbeat_publisher = Publisher('heartbeat_handler', String, queue_size=10)
        # Define consensus publisher
        self.consensus_publisher = Publisher('consensus_handler', String, queue_size=10)
        #Define sync publisher
        self.sync_publisher = Publisher('sync_handler', String, queue_size=10)
        #define sessions service proxy
        loginfo(f"{self.node_id}: NetworkInterface:Initializing sessions service")
        self.sessions = ServiceProxy('sessions/call', FunctionCall,True)
        self.sessions.wait_for_service()
        #define key store proxy
        loginfo(f"{self.node_id}: NetworkInterface:Initializing key store service")
        self.key_store = ServiceProxy('key_store/call', FunctionCall)
        self.key_store.wait_for_service()
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk = keys["pk"]
        self.sk = keys["sk"]
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
        self.queue.put({"type":type,"data":json.dumps(message.data)})
     
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    def verify_data(self,message):
        #check if message has session id
        if message["session_id"] == "": 
            #the message has no session id, so it's discovery message
            #verify the message hash 
            buff = message["message"]
            msg_signature = buff.pop('signature')
            msg_data=buff
            #check if message is string
            if type(message.message["message"]) is  str:
                #the message is a string, so it's encrypted discovery message
                #check if the node does not have active discovery session with the sender
                session = self.make_function_call(self.sessions,"get_discovery_session",message.message["node_id"])     
                #decrypt the message
                try:
                    decrypted_data = EncryptionModule.decrypt(message.message["message"],self.sk)
                    #parse the message
                    decrypted_data = json.loads(decrypted_data)
                except Exception as e:
                    if self.DEBUG:    
                        loginfo(f"{self.node_id}: error decrypting and parsing data : {e}")
                    return None
                #validate the message
                message.message["message"] = decrypted_data
                if not session: 
                    #check if public key in decrypted message
                    if decrypted_data["data"].get("pk"):
                        session = {"pk":decrypted_data["data"]["pk"]}
                    else:
                        if self.DEBUG:
                            loginfo(f"{self.node_id}: public key not found in decrypted message")
                        return None
                
            else:
                #the message is not a string, so it's not encrypted discovery message
                session = {"pk":message.message["message"]["data"]["pk"]}
                return message.message
            #verify the message signature
            if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
                if self.DEBUG:    
                    loginfo(f"{self.node_id}: signature not verified")
                return None
        else:
            #get session
            session = self.sessions("get_connection_sessions",json.dumps([message.message["session_id"]]))
            if not session:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: Invalid session")
                return

            #decrypt message
            try:
                decrypted_msg = EncryptionModule.decrypt_symmetric(message.message["message"],session["key"])
            except:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: Invalid key")
                return
            #validate message
            message.message["message"] = json.loads(decrypted_msg)
            #check counter
            if message.message["message"]["counter"]<session["counter"]:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: Invalid counter")
                return
            
            return message.message

    def send_message(self,msg_type, target, message,signed=False):
        #define target sessions
        if target == "all":
            node_ids = self.sessions("get_active_nodes",json.dumps([]))
        elif type(target) == list:
            node_ids = target
        else:
            node_ids = [target]
        #iterate over target sessions
        for node_id in node_ids:
            #check if node_id is local node_id 
            if node_id == self.node_id:
                continue
            #get node_ids session 
            session = self.make_function_call(self.sessions,"get_connection_session_by_node_id",node_id)
            if session == None:
                #check if there is discovery session
                session = self.make_function_call(self.sessions,"get_discovery_session",node_id)
                if session:
                    #stringify message data
                    msg_data = json.dumps(message["message"])
                    #encrypt message data
                    prepared_message = EncryptionModule.encrypt(msg_data,EncryptionModule.reformat_public_key(session["pk"]))
                else:
                    prepared_message= message["message"]
            else:
                #prepare message data
                msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                    "data":message
                    })
                #stringify message data
                msg_data = json.dumps(msg_data)
                #encrypt message data
                prepared_message = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
            #prepare message payload
            msg_payload = OrderedDict({
                "type": msg_type,
                "time":mktime(datetime.datetime.now().timetuple()),
                "node_id": self.node_id,
                "node_type": self.node_type,
                "session_id": message["session_id"],
                "message": prepared_message
                })
            #check if signed 
            if signed:
                msg_payload["signature"] = EncryptionModule.sign(json.dumps(msg_payload),self.sk)
            #add message to the queue
            self.publisher.publish(json.dumps({
                "target": session["node_id"],
                "time":mktime(datetime.datetime.now().timetuple()),
                "message": msg_payload,
            }))
            
    def handle_message(self, message):
        #check message type
        message =self.verify_data(message)
        if not message:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid message")
            return
        #handle message                      
        if message["node_id"]==self.node_id:
            return
        elif str(message["type"]).startswith("discovery"):
            self.discovery_publisher.publish(json.dumps(message))
        elif str(message["type"]).startswith("heartbeat"):
            self.heartbeat_publisher.publish(json.dumps(message))
        elif str(message["type"]).startswith("sync"):
            self.sync_publisher.publish(json.dumps(message))
        elif message["type"]=="data_exchange":
            self.consensus_publisher.publish(json.dumps(message))
        else:
            if self.DEBUG:
                loginfo(f"{self.node_id}: unknown message type {message['type']}")
        
    
if __name__ == "__main__":
    ns = get_namespace()
    try :
        node_id= get_param(f'{ns}/roschain/node_id') # node_name/argsname
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")
    
    try:
        node_type= get_param(f'{ns}/roschain/node_type') # node_name/argsname
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_type")
    
    network = NetworkInterface(node_id,node_type)
    rate = Rate(10)
    while not is_shutdown():
        if not network.queue.empty():
            message = network.queue.get()
            if message["type"] == "handle":
                network.handle_message(message["data"])
            elif message["type"] == "prepare":
                network.send_message(message["data"]["type"],message["data"]["target"],message["data"]["message"],message["data"].get("signed",False))
            else:
                loginfo(f"{network.node_id}: Invalid message type on network node")
        rate.sleep()
   