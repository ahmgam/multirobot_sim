from network import NetworkInterface
from connector import RabbitMQCommunicationModule
from session import SessionManager
from encryption import EncryptionModule
from queues import QueueManager
from heartbeat import HeartbeatProtocol
from discovery import DiscoveryProtocol
from messages import *
from consensus import SBFT
from blockchain import Blockchain
import rospy
from gazebo_msgs.srv import GetModelState
#from multirobot_sim.srv import GetBCRecords,SubmitTransaction

class RosChain:
    def __init__(self,node_id,node_type,endpoint,port,secret_key,auth=None,DEBUG=False):
        '''
        Initialize network interface
        '''
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
        #check if key pairs is available
        self.pk, self.sk = EncryptionModule.load_keys(f'{self.node_id}_pk.pem', f'{self.node_id}_sk.pem')
        #if not, create new public and private key pair
        if self.pk == None:
            self.pk, self.sk = EncryptionModule.generate_keys()
            EncryptionModule.store_keys(f'{self.node_id}_pk.pem', f'{self.node_id}_sk.pem',self.pk,self.sk)
        #define communication module
        self.comm = RabbitMQCommunicationModule(self.node_id,self.endpoint,self.port,self.auth,self.DEBUG)
        #define session manager
        self.sessions = SessionManager(self)
        #define queue
        self.queues = QueueManager()
        #define network interface
        self.network = NetworkInterface(self)
        #define heartbeat protocol
        self.heartbeat = HeartbeatProtocol(self)
        #define discovery protocol
        self.discovery = DiscoveryProtocol(self)
        #define blockchain
        self.blockchain = Blockchain(self)
        #define consensus protocol
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
        #self.get_record_service = rospy.Service(f'{self.node_id}/get_records',GetNewRecords,lambda req: self.get_records(self,req))
        #define submit message service
        #self.submit_message_service = rospy.Service(f'{self.node_id}/submit_message',SubmitMessage,lambda req: self.submit_message(self,req))
        #define ros node
        self.node = rospy.init_node("roschain", anonymous=True)
           
    @staticmethod
    def submit_message(self,message):
        '''
        Send message to the given public key
        '''
        #payload 
        payload = {
            "message":message,
            "source":self.node_id
        }
        #add message to the parent queue
        self.comm.buffer.put({
            "message":payload,
            "type":"consensus"   
        })
        return True

    @staticmethod
    def get_records(self,last_record):
        records = []
        for id in range(last_record,self.blockchain.db.get_last_id("blockchain")+1):
            meta,data = self.blockchain.get_transaction(id)
            records.append(json.dumps({
                f"{id}":{"meta":meta,"data":data}
            }))
        return records
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
        while True:
            #update robot position
            #check if there is any message in comm buffer
            while self.comm.is_available():
                comm_buffer =self.comm.get()
                self.queues.put_queue(comm_buffer["message"],comm_buffer["type"])
            #get message from queue
            try:
                message_buffer = self.queues.pop_queue()
                
                if message_buffer:
                    #check message type
                    if str(message_buffer["type"]) == "incoming":
                        message =Message(message_buffer["message"]) 
                        if message.message["node_id"]==self.node_id:
                            continue
                        elif message.message["type"].startswith("discovery"):
                            self.discovery.handle(message)
                        elif message.message["type"].startswith("heartbeat"):
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
                                rospy.loginfo(f"unknown message type {message.message['type']}")
                    elif str(message_buffer["type"]) == "outgoing":
                        try:
                            self.comm.send(message_buffer["message"])
                        except Exception as e:
                            if self.DEBUG:
                                rospy.loginfo(e)
                    elif str(message_buffer["type"]) == "consensus":
                        self.consensus.send(message_buffer['message'])
                    else:
                        if self.DEBUG:
                            rospy.loginfo(f'unknown message type {message_buffer["type"]}')
            except Exception as e:
                if self.DEBUG:
                    rospy.loginfo(f"error in handling message: {e}")
                continue
            #check if there is any message in output queue
            output_buffer = self.queues.pop_output_queue()
            if output_buffer:
                try:
                    self.blockchain.add_transaction(output_buffer["message"]["table"],output_buffer["message"]["data"])
                except Exception as e:
                    if self.DEBUG:
                        rospy.loginfo(e)
            #start cron
            self.cron()
                
             

if __name__ == "__main__":         


    ns = rospy.get_namespace()

    try :
        node_id= rospy.get_param(f'{ns}/roschain/node_id') # node_name/argsname
        rospy.loginfo("ROSCHAIN:Getting node_id argument, and got : ", node_id)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_id")

    try :
        node_type= rospy.get_param(f'{ns}/roschain/node_type') # node_name/argsname
        rospy.loginfo("ROSCHAIN:Getting node_type argument, and got : ", node_type)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_type")
    
    try :
        rabbitmq_endpoint= rospy.get_param(f'{ns}/roschain/rabbitmq_endpoint') # node_name/argsname
        rospy.loginfo("ROSCHAIN:Getting rabbitmq_endpoint argument, and got : ", rabbitmq_endpoint)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_endpoint")
    
    try :
        rabbitmq_username= rospy.get_param(f'{ns}/roschain/rabbitmq_username') # node_name/argsname
        rospy.loginfo("ROSCHAIN:Getting rabbitmq_username argument, and got : ", rabbitmq_username)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_username")
    
    try :
        rabbitmq_password= rospy.get_param(f'{ns}/roschain/rabbitmq_endpoint') # node_name/argsname
        rospy.loginfo("ROSCHAIN:Getting rabbitmq_password argument, and got : ", rabbitmq_password)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_password")
    
    try :
        rabbitmq_port= rospy.get_param(f'{ns}/roschain/rabbitmq_port') # node_name/argsname
        rospy.loginfo("ROSCHAIN:Getting rabbitmq_port argument, and got : ", rabbitmq_port)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_port")
    
    try :
        secret= rospy.get_param(f'{ns}/roschain/secret') # node_name/argsname
        rospy.loginfo("ROSCHAIN:Getting secret argument, and got : ", secret)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : secret")
    
    auth = {
        "username": rabbitmq_username,
        "password":rabbitmq_password
    }
  
    node = RosChain(node_id,node_type,rabbitmq_endpoint,rabbitmq_port,secret,auth,True)
  
    while not rospy.is_shutdown():
        #pop message from output queue
        node.loop()
    
            