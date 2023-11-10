from messages import *
import datetime
from encryption import EncryptionModule
from rospy import loginfo
from time import mktime
class NetworkInterface:
    
    def __init__(self,parent):
        '''
        Initialize network interface
        '''
        #define parent
        self.parent = parent
        #define discovery interval
        self.discovery_interval = 10
        #define heartbeat interval
        self.heartbeat_interval = 5
     

    def verify_data(self,message):
        #get session
        session = self.parent.sessions.get_connection_sessions(message.message["session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.parent.node_id}: Invalid session")
            return

        #decrypt message
        try:
            decrypted_msg = EncryptionModule.decrypt_symmetric(message.message["message"],session["key"])
        except:
            if self.parent.DEBUG:
                loginfo(f"{self.parent.node_id}: Invalid key")
            return
        #validate message
        message.message["message"] = json.loads(decrypted_msg)
        #check counter
        if message.message["message"]["counter"]<session["counter"]:
            if self.parent.DEBUG:
                loginfo(f"{self.parent.node_id}: Invalid counter")
            return
        
        return message.message

    def send_message(self, target, message):
        
        #define target sessions
        if target == "all":
            node_ids = self.parent.sessions.get_active_nodes()
        elif type(target) == list:
            node_ids = target
        else:
            node_ids = [target]
        #iterate over target sessions
        for node_id in node_ids:
            #check if node_id is local node_id 
            if node_id == self.parent.node_id:
                continue
            #check if session is available
            if not self.parent.sessions.has_active_connection_session(node_id):
                if self.parent.DEBUG:
                    loginfo(f"{self.parent.node_id}: No active session")
                #return Response("No active session", status=400)
            #get session
            session = self.parent.sessions.get_connection_session_by_node_id(node_id)
            #prepare message data
            msg_data = OrderedDict({
            "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":message
                })
            #stringify message data
            msg_data = json.dumps(msg_data)
            #encrypt message data
            encrypted_data = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
            #prepare message payload
            msg_payload = OrderedDict({
                "type": "data_exchange",
                "time":mktime(datetime.datetime.now().timetuple()),
                "node_id": self.parent.node_id,
                "node_type": self.parent.node_type,
                "data": msg_data,
                "pos": self.parent.pos,
                "port": self.parent.port,
                "session_id": session["session_id"],
                "message": encrypted_data
                })
            #add message to the queue
            self.parent.queues.put_queue({
                "target": session["node_id"],
                "time":mktime(datetime.datetime.now().timetuple()),
                "message": msg_payload,
                "pos": self.parent.pos,
            },"outgoing")

