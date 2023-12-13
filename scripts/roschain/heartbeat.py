from collections import OrderedDict
from encryption import EncryptionModule
from queue import Queue
import datetime
import json
from time import mktime
from rospy import loginfo,ServiceProxy,init_node,Publisher,get_namespace,spin,get_param,is_shutdown,Rate,ROSInterruptException,Subscriber
from std_msgs.msg import String
from multirobot_sim.srv import FunctionCall
class HeartbeatProtocol:
    
    def __init__(self,node_id,node_type,DEBUG=True):
        self.node_id = node_id
        self.node_type = node_type
        self.DEBUG = DEBUG
        #define heartbeat interval
        self.heartbeat_interval = 5
        #define node
        self.node = init_node("heartbeat_protocol", anonymous=True)
        #define sessions
        self.sessions = ServiceProxy('sessions/call', FunctionCall)
        #define blockchain
        self.blockchain = ServiceProxy('blockchain/call', FunctionCall)
        #define key store proxy
        self.key_store = ServiceProxy('key_store/call', FunctionCall)
        #define heartbeat subscriber
        self.subscriber = Subscriber('heartbeat_handler', String, self.to_queue)
        #messsage publisher
        self.publisher = Publisher('send_message', String, queue_size=10)
        #define network 
        self.network = ServiceProxy("network/call",FunctionCall)
        #define last heartbeat
        self.last_call = mktime(datetime.datetime.now().timetuple())
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk = keys["pk"]
        self.sk = keys["sk"]
        #define queue
        self.queue = Queue()
           
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    
    def to_queue(self,data):
        self.queue.put(json.loads(data.data))
    
    def cron(self):
        #send heartbeat to all nodes
        for session_id, session in self.make_function_call(self.sessions,"get_connection_sessions").items():
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
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting handle_heartbeat")
            self.handle_heartbeat(message)
        elif message["type"] == "heartbeat_response":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting handle_heartbeat_response")
            self.handle_heartbeat_response(message)
        else:
            if self.DEBUG:
                loginfo(f"{self.node_id}: unknown message type {message['type']}")
                
    def send_heartbeat(self,session):
        
        #send heartbeat to session
        #prepare message 
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": session["counter"]+1,
                "data":self.make_function_call(self.sessions,"get_node_state_table"),
                "blockchain_status":self.make_function_call(self.blockchain,"get_sync_info")
            })
        #serialize message
        msg_data= json.dumps(msg_data)
        #call network service
        self.make_function_call(self.network,"send_message",session["node_id"],msg_data)
        
           
    def handle_heartbeat(self,message):
        #receive heartbeat from node
        #get session
        session = self.make_function_call(self.sessions,"get_connection_sessions",*message["session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid session")
            return
        #get message hash and signature
        buff = message.copy()
        msg_signature = buff.pop("signature")
        #serialize message buffer
        msg_data= json.dumps(buff)
        #verify message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid signature")
            return
        #decrypt message
        try:
            decrypted_msg = EncryptionModule.decrypt_symmetric(message["message"],session["key"])
        except:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid key")
            return
        #validate message
        message["message"] = json.loads(decrypted_msg)
        #check counter
        #if message["message"]["counter"]<=session["counter"]:
        #    if self.DEBUG:
        #        loginfo(f"{self.node_id}: Invalid counter")
        #    return
        #update node state table
        #self.parent.server.logger.warning(f'table request : {json.dumps(message["message"]["data"])}' )
        self.make_function_call(self.sessions,"update_node_state_table",message["message"]["data"])
        #chcek blockchain status
        if self.make_function_call(self.blockchain,"check_sync",*message["message"]["blockchain_status"]) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Un synced blockchain, sending sync request")
            self.make_function_call(self.blockchain,"send_sync_request")
            
        #prepare message 
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": session["counter"]+1,
                "data":self.make_function_call(self.sessions,"get_node_state_table"),
                "blockchain_status":self.make_function_call(self.blockchain,"get_sync_info")
            })
        #serialize message
        msg_data= json.dumps(msg_data)
        #call network service
        self.make_function_call(self.network,"send_message",session["node_id"],msg_data)
 
    def handle_heartbeat_response(self,message):
        #receive heartbeat from node
        #get session
        session = self.make_function_call(self.sessions,"get_connection_sessions",message["session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid session")
            return
        
        #get message hash and signature
        buff = message.copy()
        msg_signature = buff.pop("signature")
        #serialize message buffer
        msg_data= json.dumps(buff)
        #verify message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid signature")
            return
        #decrypt message
        try:
            decrypted_msg = EncryptionModule.decrypt_symmetric(message["message"],session["key"])
        except:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid key")
            return
        #validate message
        message["message"] = json.loads(decrypted_msg)
        #self.parent.server.logger.warning(f'table response : {json.dumps(message["message"]["data"])}' )
        #check counter
        #if message["message"]["counter"]<=session["counter"]:
        #    if self.DEBUG:
        #        loginfo(f"{self.node_id}: Invalid counter")
        #    return
        #update node state table
        self.make_function_call(self.sessions,"update_node_state_table",message["message"]["data"])
        #update session
        self.make_function_call(self.sessions,"update_connection_session",message["session_id"],{
            "counter":message["message"]["counter"],
            "last_active": mktime(datetime.datetime.now().timetuple())})
        #chcek blockchain status
        if self.make_function_call(self.blockchain,"check_sync",*message["message"]["blockchain_status"])== False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Un synced blockchain, sending sync request")
            self.make_function_call(self.blockchain,"send_sync_request")    
            
if __name__ == '__main__':
    ns = get_namespace()
    
    try :
        node_id= get_param(f'{ns}/discovery/node_id') # node_name/argsname
        loginfo(f"discovery: Getting node_id argument, and got : {node_id}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")
    
    try :
        node_type= get_param(f'{ns}/discovery/node_type') # node_name/argsname
        loginfo(f"discovery: Getting endpoint argument, and got : {node_type}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_type")
    
    node = HeartbeatProtocol(node_id,node_type,DEBUG=True)
    rate = Rate(10)
    while not is_shutdown():
        #check queue
        if not node.queue.empty():
            msg = node.queue.get()
            node.handle(msg)
        rate.sleep()