
import json
import datetime
from std_srvs.srv import Trigger, TriggerResponse
import rospy
from time import mktime
from multirobot_sim.srv import GetBCRecords,SubmitTransaction,GetBCRecordsResponse,SubmitTransactionResponse, DatabaseQuery, DatabaseQueryRequest
from encryption import EncryptionModule
from heartbeat import HeartbeatProtocol
from network import NetworkInterface
from session import SessionManager
from queues import QueueManager
from discovery import DiscoveryProtocol
from blockchain import Blockchain
from consensus import SBFT
from connector import MQTTCommunicationModule
from messages import Message
#from multirobot_sim.srv import GetBCRecords,SubmitTransaction
#####################################
# RosChain Module
#####################################

class RosChain:
    def __init__(self,node_id,node_type,endpoint,port,secret_key,base_directory,auth=None,DEBUG=False):
        '''
        Initialize network interface
        '''
        #define ros node
        self.node = rospy.init_node("roschain", anonymous=True)
        #define is_initialized
        self.ready = False
        #define debug mode
        self.DEBUG = DEBUG
        #define secret 
        self.secret_key = secret_key
        #define dummy position
        #self.service_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        #self.service_proxy.wait_for_service()
        self.pos = "0,0,0"
        #define node id
        self.node_id = node_id
        #define node type
        self.node_type = node_type
        #get port from parent
        self.endpoint = endpoint
        #define auth
        self.auth = auth
        #define port
        self.port = port
        #define base directory
        self.base_directory = base_directory
        #define block size 
        self.block_size = 10
        #define block tolerance 
        self.block_tolerance = 5
        #check if key pairs is available
        rospy.loginfo(f"{self.node_id}: ROSChain:Checking if key pairs are available")
        self.pk, self.sk = EncryptionModule.load_keys(f'{self.base_directory}/{self.node_id}_pk.pem', f'{self.base_directory}/{self.node_id}_sk.pem')
        #if not, create new public and private key pair
        if self.pk == None:
            rospy.loginfo(f"{self.node_id}: ROSChain:Key pairs are not available, creating new")
            self.pk, self.sk = EncryptionModule.generate_keys()
            EncryptionModule.store_keys(f'{self.node_id}_pk.pem', f'{self.node_id}_sk.pem',self.pk,self.sk)
        #define communication module
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing communication module")
        self.comm = MQTTCommunicationModule(self.node_id,self.endpoint,self.port,self.auth,self.DEBUG)
        #define session manager
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing session manager")
        self.sessions = SessionManager(self)
        #define queue
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing queue manager")
        self.queues = QueueManager()
        #define network interface
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing network interface")
        self.network = NetworkInterface(self)
        #define heartbeat protocol
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing heartbeat protocol")
        self.heartbeat = HeartbeatProtocol(self)
        #define discovery protocol
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing discovery protocol")
        self.discovery = DiscoveryProtocol(self)
        #define blockchain
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing blockchain")
        self.blockchain = Blockchain(self)
        #define consensus protocol
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing consensus protocol")
        self.consensus = SBFT(self)
        #cron interval
        self.cron_interval = 1
        #cron procedure list
        self.cron_procedures = []
        #register cron procedures
        self.cron_procedures.append(self.heartbeat.cron)
        self.cron_procedures.append(self.discovery.cron)
        self.cron_procedures.append(self.consensus.cron)
        self.cron_procedures.append(self.blockchain.cron)
        #define records service
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing records service")
        self.get_record_service = rospy.Service(f'get_records',GetBCRecords,lambda req: self.get_records(req))
        #define submit message service
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing submit message service")
        self.submit_message_service = rospy.Service(f'submit_message',SubmitTransaction,lambda req: self.submit_message(self,req))
        #define is_initialized service
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing is_initialized service")
        self.get_status_service = rospy.Service(f'get_status',Trigger,lambda req: self.get_status(req))
        #node is ready
        self.ready = True

    def get_status(self,args):

        if len(self.sessions.get_active_nodes()) > 0:
            tag = "CONNECTED"
        else:
            if self.ready:
                tag = "READY"
            else:
                tag = "STARTING"
        return TriggerResponse(self.ready,tag)
    
    @staticmethod           
    def submit_message(self,args):
        '''
        Send message to the given public key
        '''
        rospy.loginfo(f"{self.node_id}: Task_allocator: {self.node_id} is sending message of type {args.table_name}")
        table_name = args.table_name
        data = args.message
        msg_time = mktime(datetime.datetime.now().timetuple())
        message = {
            "table_name":table_name,
            "data":data,
            "time":datetime.datetime.fromtimestamp(msg_time).strftime("%Y-%m-%d %H:%M:%S") 
        }
        #payload 
        payload = {
            "message":message,
            "source":self.node_id,
            "timestamp":msg_time
        }
        #add message to the parent queue
        self.comm.buffer.put({
            "message":payload,
            "time":msg_time,
            "type":"consensus"   
        })
        return SubmitTransactionResponse("Success")

    def get_records(self,last_record):
        records = []
        try:
            for id in range(last_record.last_trans_id,self.blockchain.db.get_last_id("blockchain")+1):
                meta,data = self.blockchain.get_transaction(id)
                records.append(json.dumps({
                    f"{id}":{"meta":meta,"data":data}
                }))
        except:
            records = []
        return GetBCRecordsResponse(records)
    def cron(self):
        for procedure in self.cron_procedures:
            procedure()
            
    def update_pos(self):
        pass
        #self.pose = self.service_proxy(self.node_id, 'world').pose

    def loop(self):
        '''
        start listening for incoming connections
        '''
        #handle consensus queue
        self.handle_consensus_queue()
        #check if there is any message in comm buffer
        self.handle_communication_queue()
        #check if there is any message in output queue
        self.handle_output_queue()
        #handle all cron routines
        self.cron()
                
    def handle_communication_queue(self):
        #check if there is any message in comm buffer
        while self.comm.is_available():
            comm_buffer =self.comm.get()
            self.queues.put_queue(comm_buffer["message"],comm_buffer["type"])
        #get message from queue
        message_buffer = self.queues.pop_queue()
        
        if message_buffer:
            #check message type
            if str(message_buffer["type"]) == "incoming":
                message =Message(message_buffer["message"])                         
                if message.message["node_id"]==self.node_id:
                    return
                elif str(message.message["type"]).startswith("discovery"):
                    self.discovery.handle(message)
                elif str(message.message["type"]).startswith("heartbeat"):
                    self.heartbeat.handle(message)
                elif message.message["type"]=="data_exchange":
                    #for test purposes
                    data = self.network.verify_data(message)
                    if data:
                        pass
                        #self.network.server.logger.warning(f"Message from {data['node_id']} : {data['message']}")
                        self.consensus.handle(data)
                else:
                    if self.DEBUG:
                        rospy.loginfo(f"{self.node_id}: unknown message type {message.message['type']}")
            elif str(message_buffer["type"]) == "outgoing":
                self.comm.send(message_buffer["message"])
            elif str(message_buffer["type"]) == "consensus":
                self.consensus.send(message_buffer['message'])
            else:
                if self.DEBUG:
                    rospy.loginfo(f'{self.node_id}: unknown message type {message_buffer["type"]}')
      
    def handle_output_queue(self):
        #check if there is any message in output queue
        if self.queues.output_queue_count > 10:
                #get a list of transactions
                transactions = []
                for _ in range(self.block_size):
                    transactions.append(self.queues.pop_output_queue())
                self.blockchain.add_block(transactions)
                #if output_buffer:
                #    if type(output_buffer["message"]["data"]) == str:
                #        output_buffer["message"]["data"] = json.loads(output_buffer["message"]["data"])
                #    self.blockchain.add_transaction(output_buffer["message"]["table_name"],output_buffer["message"]["data"])
                    #try:
                        
                    #except Exception as e:
                    #    if self.DEBUG:
                    #        rospy.loginfo(e)
              
    def handle_consensus_queue(self):
        #check if there is any message in consensus queue
        if self.queues.consensus_queue_count >= self.block_size + self.block_tolerance:
            #get a list of transactions
            transactions = []
            for _ in range(self.block_size):
                transactions.append(self.queues.pop_output_queue())
            payload = {
                "message":json.dumps(transactions),
                "source":self.node_id,
                "timestamp":mktime(datetime.datetime.now().timetuple())
            }
            #add message to the parent queue
            self.comm.buffer.put({
                "message":payload,
                "time":mktime(datetime.datetime.now().timetuple()),
                "type":"consensus"   
            })
#####################################
# Main
#####################################             

if __name__ == "__main__":         
    ns = rospy.get_namespace()
    try :
        node_id= rospy.get_param(f'{ns}/roschain/node_id') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting node_id argument, and got : ", node_id)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_id")

    try :
        node_type= rospy.get_param(f'{ns}/roschain/node_type') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting node_type argument, and got : ", node_type)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_type")
    
    try :
        rabbitmq_endpoint= rospy.get_param(f'{ns}/roschain/rabbitmq_endpoint') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_endpoint argument, and got : ", rabbitmq_endpoint)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_endpoint")
    
    try :
        rabbitmq_username= rospy.get_param(f'{ns}/roschain/rabbitmq_username') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_username argument, and got : ", rabbitmq_username)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_username")
    
    try :
        rabbitmq_password= rospy.get_param(f'{ns}/roschain/rabbitmq_endpoint') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_password argument, and got : ", rabbitmq_password)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_password")
    
    try :
        rabbitmq_port= rospy.get_param(f'{ns}/roschain/rabbitmq_port') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_port argument, and got : ", rabbitmq_port)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_port")
    
    try :
        secret= rospy.get_param(f'{ns}/roschain/secret') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting secret argument, and got : ", secret)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : secret")
    
    try :
        base_directory= rospy.get_param(f'{ns}/roschain/base_directory') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting base_directory argument, and got : ", base_directory)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : base_directory")
    auth = {
        "username": rabbitmq_username,
        "password":rabbitmq_password
    }
    auth = None
    
    node = RosChain(node_id,node_type,rabbitmq_endpoint,int(rabbitmq_port),secret,base_directory,auth,True)
  
    while not rospy.is_shutdown():
        #pop message from output queue
        node.loop()
    
            