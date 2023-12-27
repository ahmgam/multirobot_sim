#!/usr/bin/env python
from messages import *
from time import sleep,mktime
import datetime
from random import choices
from string import ascii_lowercase
from encryption import *
from rospy import loginfo,init_node,Publisher,Subscriber,ServiceProxy,Rate,is_shutdown,get_namespace,get_param,ROSInterruptException
from multirobot_sim.srv import FunctionCall
from std_msgs.msg import String
from queue import Queue

########################################
# Discovery protocol
########################################

class DiscoveryProtocol:
    def __init__(self,node_id,node_type,secret,DEBUG=True):
        #define node id
        self.node_id = node_id
        #define node type
        self.node_type = node_type
        #define secret
        self.secret = secret
        #define debug mode
        self.DEBUG = DEBUG
        #define node
        self.node = init_node("discovery", anonymous=True)
        #define discovery interval
        self.discovery_interval = 10
        #define discovery last call
        self.last_call = mktime(datetime.datetime.now().timetuple())
        #publisher
        loginfo(f"{self.node_id}: Discovery:Initializing publisher and subscriber")
        self.publisher = Publisher('send_message', String, queue_size=10)
        #subscriber 
        self.subscriber = Subscriber('discovery_handler', String, self.put_queue)
        #define session
        loginfo(f"{self.node_id}: Discovery:Initializing session service")
        self.sessions = ServiceProxy('sessions/call', FunctionCall,True)
        self.sessions.wait_for_service()
        #define key store proxy
        loginfo(f"{self.node_id}: Discovery:Initializing key store service")
        self.key_store = ServiceProxy('key_store/call', FunctionCall)
        self.key_store.wait_for_service()
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk = keys["pk"]
        self.sk = keys["sk"]
        # queue
        self.queue = Queue()
        loginfo(f"{self.node_id}: Discovery:Initialized successfully")
        
    def cron(self):
        #check if disvoery last call is more than discovery interval
        #loginfo(f"session time : {mktime(datetime.datetime.now().timetuple()) - self.last_call}")
        if mktime(datetime.datetime.now().timetuple()) - self.last_call > self.discovery_interval:
            #update last call
            self.last_call = mktime(datetime.datetime.now().timetuple())
            #start discovery
            self.discover()
            
    def put_queue(self,message):
        self.queue.put(json.loads(message))
        
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)

    def handle(self,message):
        if message.message["type"] == "discovery_request":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting response_to_discovery")
            self.respond_to_discovery(message)
        elif message.message["type"] == "discovery_response":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting verify_discovery")
            self.verify_discovery(message)
        elif message.message["type"] == "discovery_verification":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting verify_discovery_response")
            self.verify_discovery_response(message)
        elif message.message["type"] == "discovery_verification_response":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting approve_discovery")
            self.approve_discovery(message)
        elif message.message["type"] == "discovery_approval":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting approve_discovery_response")
            self.approve_discovery_response(message)
        elif message.message["type"] == "discovery_approval_response":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting finalize_discovery")
            self.finalize_discovery(message)
        else:
            loginfo(f"{self.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, but no handler found")
    ################################
    # Challenge management
    ################################   
    def generate_challenge(self, length=20):
        return ''.join(choices(ascii_lowercase, k=length))
    
    def solve_challenge(self,challenge):
        solution = EncryptionModule.hash(challenge+self.secret)
        client_sol = solution[0:len(solution)//2]
        server_sol = solution[len(solution)//2:]
        return client_sol, server_sol
      
    ################################
    # discovery protocol
    ################################
    def discover(self):
        #discover new nodes on the network
        self.publisher.publish(json.dumps({
            "target": "all",
            "time":mktime(datetime.datetime.now().timetuple()),
            "message": EncryptionModule.format_public_key(self.pk),
            "signed":True}))

    def respond_to_discovery(self,message):
        #respond to discovery requests and send challenge
        #first verify the message
        try:
            message = DiscoveryMessage(message.message) 
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: validation error {e}")
            return None
        #check if the node has active connection session with the sender        
        if self.make_function_call(self.sessions,"has_active_connection_session",message.message["node_id"]):
            if self.DEBUG:
                loginfo(f"{self.node_id}: connection session is already active") 
            return None
        #check if the node has active discovery session with the sender
        if self.make_function_call(self.sessions,"get_discovery_session",message.message["node_id"]):
            if self.DEBUG:    
                loginfo(f"{self.node_id}: discovery session is already active")
            return None
        else:
            #create new session
            session_data = {
                "pk": message.message["message"]["data"]["pk"],
                "role":"server",
                "counter": message.message["message"]["counter"],
                "node_type": message.message["node_type"],     
            }
            self.make_function_call(self.sessions,"create_discovery_session",message.message["node_id"],session_data)
        #prepare discovery response message
        msg_data ={
            "pk": EncryptionModule.format_public_key(self.pk)
            }
        #send the message
        self.publisher.publish(json.dumps({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": msg_data,
                                      "signed":True}))
    
    def verify_discovery(self,message):
        #verify discovery request and send challenge response
        #check if the node is already connected to the network
        if self.make_function_call(self.sessions,"has_active_connection_session",message.message["node_id"]):
            if self.DEBUG:    
                loginfo(f"{self.node_id}: connection session is already active")
            return None
        #first verify the message     
        try :
            message=DiscoveryResponseMessage(message)
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error validating message : {e}")
            return None
        try:
            #generate challenge random string
            challenge = self.generate_challenge()
            #solve the challenge
            client_sol, server_sol = self.solve_challenge(challenge)
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error generating challenge : {e}")
            return None
        #create discovery session
        session_data = {
            "pk": message.message["message"]["data"]["pk"],
            "role": "client",
            "node_type": message.message["node_type"],
            "challenge": challenge,
            "client_challenge_response": client_sol,
            "server_challenge_response": server_sol
        }
        #create discovery session
        self.make_function_call(self.sessions,"create_discovery_session",message.message["node_id"],session_data)
        #prepare verification message 
        msg_data = {
            "challenge": challenge,
            "client_challenge_response": client_sol
            }
        #send the message
        self.publisher.publish(json.dumps({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": msg_data,
                                      "signed":True}))
 
    def verify_discovery_response(self,message):
        #verify discovery response and add node to the network
        #check if the node is already connected to the network
        if self.make_function_call(self.sessions,"has_active_connection_session",message.message["node_id"]):
            if self.DEBUG:
                loginfo(f"{self.node_id}: connection session is already active")
            return None
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session",message.message["node_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #verify the message s
        try :
            message=VerificationMessage(message.message)
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error validating message : {e}")
            return None
        
        #get the challenge from the incoming message
        challenge = message.message["message"]["data"]["challenge"]
        #solve the challenge
        client_sol, server_sol = self.solve_challenge(challenge)
        #compare the client challenge response
        if message.message["message"]["data"]["client_challenge_response"] != client_sol:
            if self.DEBUG:
                loginfo(f"{self.node_id}: client challenge response not verified")
            return None
        #update discovery session
        session_data = {
            "pk": session["pk"],
            "role": "server",
            "node_type": message.message["node_type"],
            "challenge": challenge,
            "client_challenge_response": client_sol,
            "server_challenge_response": server_sol
        }
        #update discovery session
        self.make_function_call(self.sessions,"update_discovery_session",message.message["node_id"],session_data)
        #prepare verification message
        msg_data = {
            "challenge": challenge,
            "server_challenge_response": server_sol
            }
        #send the message
        self.publisher.publish(json.dumps({
            "target": message.message["node_id"],
            "time":mktime(datetime.datetime.now().timetuple()),
            "message": msg_data,
            "signed":True}))

    def approve_discovery(self,message):
        #approve discovery request and send approval response
        #check if the node is already connected to the network
        if self.make_function_call(self.sessions,"has_active_connection_session",message.message["node_id"]):
            if self.DEBUG:
                loginfo(f"{self.node_id}: connection session is already active")
            return None
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session",message.message["node_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #verify the message
        try :
            message=VerificationResponseMessage(message.message)
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error validating message : {e}")
            return None
        #compare the client challenge response
        if message.message["message"]["data"]["server_challenge_response"] != session["server_challenge_response"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: client challenge response not verified")
            return None
        
        #creating new session with symmetric key and session id
        #first generate symmetric key
        key = EncryptionModule.generate_symmetric_key()
        #get the session id
        session_id = self.make_function_call(self.sessions,"generate_session_id")
        #create new session
        session_data = {
            "pk": session["pk"],
            "node_id": message.message["node_id"],
            "node_type": message.message["node_type"],
            "last_active": mktime(datetime.datetime.now().timetuple()),
            "port": message.message["port"],
            "role": "server",   
            "session_id": session_id,
            "key": key,
            "status": "pending",
            "last_heartbeat": mktime(datetime.datetime.now().timetuple()),
            "approved": False
        }
        self.make_function_call(self.sessions,"create_connection_session",session_id,session_data)
        #prepare approval message
        msg_data ={
            "session_id": session_id,
            "session_key": key,
            "test_message": EncryptionModule.encrypt_symmetric("client_test",key)
            }
        #send the message
        self.publisher.publish(json.dumps({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": msg_data,
                                      "signed":True}))
            
    def approve_discovery_response(self,message):
        #approve discovery response and add node to the network
        #check if the node is already connected to the network
        if self.make_function_call(self.sessions,"has_active_connection_session",message.message["node_id"]):
            if self.DEBUG:
                loginfo(f"{self.node_id}: connection session is already active")
            return None
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session",message.message["node_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #validate the message
        try :
            message=ApprovalMessage(message.message)
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error validating message : {e}")
            return None
        #first generate symmetric key
        key = message.message["message"]["data"]["session_key"]
        #get the session id
        session_id = message.message["message"]["data"]["session_id"]
        #decrypt the test message
        try:
            decrypted_test = EncryptionModule.decrypt_symmetric(message.message["message"]["data"]["test_message"],key)
            if decrypted_test != "client_test":
                if self.DEBUG:
                    loginfo(f"{self.node_id}: test message not decrypted")
                return None
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error decrypting test message : {e}")
            return None
        #create new session
        session_data = {
            "pk": session["pk"],
            "node_id": message.message["node_id"],
            "node_type": message.message["node_type"],
            "last_active": mktime(datetime.datetime.now().timetuple()),
            "role": "server",   
            "session_id": session_id,
            "key": key,
            "status": "active",
            "last_heartbeat": mktime(datetime.datetime.now().timetuple()),
            "approved": True
        }
        self.make_function_call(self.sessions,"create_connection_session",session_id,session_data)
        #prepare approval message
        msg_data = {
            "session_id": session_id,
            "test_message": EncryptionModule.encrypt_symmetric("server_test",key)
            }
        #send the message
        self.publisher.publish(json.dumps({
            "target": message.message["node_id"],
            "time":mktime(datetime.datetime.now().timetuple()),
            "message": msg_data,
            "signed":True}))

    def finalize_discovery(self,message):
        #approve discovery response and add node to the network
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session",message.message["node_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #validate the message
        try :
            message=ApprovalResponseMessage(message.message)
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error validating message : {e}")
            return None
        #decrypt the test message
        try:
            decrypted_test = EncryptionModule.decrypt_symmetric(message.message["message"]["data"]["test_message"],session["key"])
            if decrypted_test != "server_test":
                if self.DEBUG:
                    loginfo(f"{self.node_id}: test message not decrypted")
                return None
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error decrypting test message : {e}")
            return None
        
        #get the session id
        session_id = message.message["message"]["data"]["session_id"]
        #update the session
        session_data = {
            "approved": True,
            "status": "active",
        }
        self.make_function_call(self.sessions,"update_connection_session",session_id,session_data)
        
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
    
    try :
        secret= get_param(f'{ns}/discovery/secret') # node_name/argsname
        loginfo(f"discovery: Getting secret argument, and got : {secret}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : secret")
    
    node = DiscoveryProtocol(node_id,node_type,secret,DEBUG=True)
    #define rate
    rate = Rate(10)
    while not is_shutdown():
        node.cron()
        #check if queue has messages
        if not node.queue.empty():
            message = node.queue.get()
            node.handle(message)
        else:
            rate.sleep()