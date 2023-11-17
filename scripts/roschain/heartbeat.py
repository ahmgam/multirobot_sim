from collections import OrderedDict
from encryption import EncryptionModule
import datetime
import json
from time import mktime
from rospy import loginfo,ServiceProxy,init_node,Publisher
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
        #messsage publisher
        self.publisher = Publisher('send_message', String, queue_size=10)
        #define last heartbeat
        self.last_call = mktime(datetime.datetime.now().timetuple())
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk = keys["pk"]
        self.sk = keys["sk"]
           
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    
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
        
        if message.message["type"] == "heartbeat_request":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting handle_heartbeat")
            self.handle_heartbeat(message)
        elif message.message["type"] == "heartbeat_response":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting handle_heartbeat_response")
            self.handle_heartbeat_response(message)
        else:
            if self.DEBUG:
                loginfo(f"{self.node_id}: unknown message type {message.message['type']}")
                
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
        #encrypt message
        encrypted_msg = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
        #create heartbeat message
        payload = OrderedDict({
            "session_id": session["session_id"],
            "node_id": self.node_id,
            "node_type": self.node_type,
            "type": "heartbeat_request",
            "time":mktime(datetime.datetime.now().timetuple()),
            "message":encrypted_msg
            })
        #serialize message
        msg_data= json.dumps(payload)
        #get message hash and signature
        msg_signature = EncryptionModule.sign(msg_data,self.sk)
        #add hash and signature to message
        payload["signature"] = msg_signature
        #send message
        self.publisher.publish(json.dumps([{"target": session["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload},"outgoing"]))
           
    def handle_heartbeat(self,message):
        #receive heartbeat from node
        #get session
        session = self.make_function_call(self.sessions,"get_connection_sessions",*message.message["session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid session")
            return
        #get message hash and signature
        buff = message.message.copy()
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
            decrypted_msg = EncryptionModule.decrypt_symmetric(message.message["message"],session["key"])
        except:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid key")
            return
        #validate message
        message.message["message"] = json.loads(decrypted_msg)
        #check counter
        #if message.message["message"]["counter"]<=session["counter"]:
        #    if self.DEBUG:
        #        loginfo(f"{self.node_id}: Invalid counter")
        #    return
        #update node state table
        #self.parent.server.logger.warning(f'table request : {json.dumps(message.message["message"]["data"])}' )
        self.make_function_call(self.sessions,"update_node_state_table",message.message["message"]["data"])
        #chcek blockchain status
        if self.make_function_call(self.blockchain,"check_sync",*message.message["message"]["blockchain_status"]) == False:
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
        
        #encrypt message
        encrypted_msg = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
        #create heartbeat message
        payload = OrderedDict({
            "session_id": session["session_id"],
            "node_id": self.node_id,
            "node_type":self.node_type,
            "type": "heartbeat_response",
            "time":mktime(datetime.datetime.now().timetuple()),
            "message":encrypted_msg
            })
        #serialize message
        msg_data= json.dumps(payload)
        #get message hash and signature
        msg_signature = EncryptionModule.sign(msg_data,self.sk)
        #add hash and signature to message
        payload["signature"] = msg_signature
        #send message
        self.publisher.publish(json.dumps([{"target": session["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload},"outgoing"]))
 
    def handle_heartbeat_response(self,message):
        #receive heartbeat from node
        #get session
        session = self.make_function_call(self.sessions,"get_connection_sessions",message.message["session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid session")
            return
        
        #get message hash and signature
        buff = message.message.copy()
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
            decrypted_msg = EncryptionModule.decrypt_symmetric(message.message["message"],session["key"])
        except:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid key")
            return
        #validate message
        message.message["message"] = json.loads(decrypted_msg)
        #self.parent.server.logger.warning(f'table response : {json.dumps(message.message["message"]["data"])}' )
        #check counter
        #if message.message["message"]["counter"]<=session["counter"]:
        #    if self.DEBUG:
        #        loginfo(f"{self.node_id}: Invalid counter")
        #    return
        #update node state table
        self.make_function_call(self.sessions,"update_node_state_table",message.message["message"]["data"])
        #update session
        self.make_function_call(self.sessions,"update_connection_session",message.message["session_id"],{
            "counter":message.message["message"]["counter"],
            "last_active": mktime(datetime.datetime.now().timetuple())})
        #chcek blockchain status
        if self.make_function_call(self.blockchain,"check_sync",*message.message["message"]["blockchain_status"])== False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Un synced blockchain, sending sync request")
            self.make_function_call(self.blockchain,"send_sync_request")    
