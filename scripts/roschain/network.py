from messages import *
import datetime
from queue import Queue
from encryption import EncryptionModule
from rospy import loginfo
from time import mktime
from rospy import init_node, Subscriber,Publisher,get_namespace,get_param,ROSInterruptException,Service,ServiceProxy,Rate,is_shutdown
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
        #define queue
        self.queue = Queue()
        #define subscriber
        self.subscriber = Subscriber('prepare_message', String, self.handle_data)
        #define publisher
        self.publisher = Publisher('send_message', String, queue_size=10)
        #define server
        self.server = Service('call', FunctionCall, self.handle_function_call)
        #define sessions service proxy
        self.sessions = ServiceProxy('sessions', FunctionCall)
        #define is_initialized
        
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
     

    def verify_data(self,message):
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

    def send_message(self, target, message):
        
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
            #check if session is available
            if not self.sessions("has_active_connection_session",json.dumps([node_id])):
                if self.DEBUG:
                    loginfo(f"{self.node_id}: No active session")
                #return Response("No active session", status=400)
            #get session
            session = self.sessions("get_connection_session_by_node_id",json.dumps([node_id]))
            #prepare message data
            msg_data = OrderedDict({
            "timestamp": str(datetime.datetime.now()),
                "counter": self.comm.counter,
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
                "node_id": self.node_id,
                "node_type": self.node_type,
                "data": msg_data,
                "session_id": session["session_id"],
                "message": encrypted_data
                })
            #add message to the queue
            self.publisher.publish(json.dumps([{
                "target": session["node_id"],
                "time":mktime(datetime.datetime.now().timetuple()),
                "message": msg_payload,
            },"outgoing"]))

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
            messae = network.queue.get()
            network.send_message(*messae)
        rate.sleep()
   