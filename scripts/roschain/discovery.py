#!/usr/bin/env python
from messages import *
from time import sleep,mktime
import datetime
from random import choices,randint
from string import ascii_lowercase
from encryption import *
from rospy import loginfo,init_node,Publisher,Subscriber,ServiceProxy,Rate,is_shutdown,get_namespace,get_param,ROSInterruptException
from multirobot_sim.srv import FunctionCall
from std_msgs.msg import String
from queue import Queue
from messages import MessagePublisher, MessageSubscriber
########################################
# Discovery protocol
########################################

class DiscoveryProtocol:
    def __init__(self,node_id,node_type,secret,max_delay,DEBUG=True):
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
        self.last_call = mktime(datetime.datetime.now().timetuple()) + randint(1,max_delay)
        #define key store proxy
        loginfo(f"{self.node_id}: Discovery:Initializing key store service")
        self.key_store = ServiceProxy(f"/{self.node_id}/key_store/call", FunctionCall)
        self.key_store.wait_for_service(timeout=100)
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk,self.sk =EncryptionModule.reconstruct_keys(keys["pk"],keys["sk"])
        #define session
        loginfo(f"{self.node_id}: Discovery:Initializing session service")
        self.sessions = ServiceProxy(f"/{self.node_id}/sessions/call", FunctionCall,True)
        self.sessions.wait_for_service(timeout=100)
        #publisher
        loginfo(f"{self.node_id}: Discovery:Initializing publisher and subscriber")
        self.publisher = MessagePublisher(f"/{self.node_id}/network/prepare_message")
        #subscriber 
        self.subscriber = MessageSubscriber(f"/{self.node_id}/discovery/discovery_handler", self.put_queue)
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
        self.queue.put(message)
        
    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    
    def except_active_session(self,node_id):
        #check if the node is already connected to the network
        if self.make_function_call(self.sessions,"has_active_connection_session",node_id):
            if self.DEBUG:    
                loginfo(f"{self.node_id}: connection session is already active")
            return None
        return True
    
    def handle(self,message):
        
        #first verify the me
        if message["type"] == "discovery_request":
            if not self.except_active_session(message["node_id"]):
                return None
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting response_to_discovery")
            self.respond_to_discovery(message)
        elif message["type"] == "discovery_response":
            if not self.except_active_session(message["node_id"]):
                return None
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting verify_discovery")
            self.verify_discovery(message)
        elif message["type"] == "discovery_verification":
            if not self.except_active_session(message["node_id"]):
                return None
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting verify_discovery_response")
            self.verify_discovery_response(message)
        elif message["type"] == "discovery_verification_response":
            #if not self.except_active_session(message["node_id"]):
            #    return None
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting approve_discovery")
            self.approve_discovery(message)
        elif message["type"] == "discovery_approval":
            #if not self.except_active_session(message["node_id"]):
            #    return None
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting approve_discovery_response")
            self.approve_discovery_response(message)
        elif message["type"] == "discovery_approval_response":
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, starting finalize_discovery")
            self.finalize_discovery(message)
        else:
            loginfo(f"{self.node_id}: Received message from {message['node_id']} of type {message['type']}, but no handler found")
    ################################
    # Challenge management
    ################################   
    def generate_challenge(self, length=20):
        #generate string which changes every 1 minues
        #first, get time related string
        time_str = str(mktime(datetime.datetime.now().timetuple())//6)
        #generate hash that are combination of time and secret
        hash_str = EncryptionModule.hash(time_str+self.secret)
        #return the first 20 characters
        return hash_str[0:length]
    
    def solve_challenge(self,challenge):
        solution = EncryptionModule.hash(challenge+self.secret)
        client_sol = solution[0:len(solution)//2]
        server_sol = solution[len(solution)//2:]
        return client_sol, server_sol
    
    def generate_session_id(self):
        #generate session id, random string of 32 characters
        return ''.join(choices(ascii_lowercase, k=32))
      
    ################################
    # discovery protocol
    ################################
    def discover(self):
        #discover new nodes on the network
        loginfo(f"{self.node_id}: Starting discovery")
        self.publisher.publish({
            "target": "all",
            "time":mktime(datetime.datetime.now().timetuple()),
            "message":{ 'pk':EncryptionModule.format_public_key(self.pk)},
            "type": "discovery_request",
            "signed":True})

    def respond_to_discovery(self,message):
        #respond to discovery requests and send challenge
        #first verify the message
        try:
            message = DiscoveryMessage(message) 
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: validation error {e}")
            return None
        #check if the node has active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session_by_node_id",message.message["node_id"])
        if session:
            if self.DEBUG:    
                loginfo(f"{self.node_id}: discovery session is already active")
            return None
        else:
            #generate session id 
            session_id = self.generate_session_id()
            #create new session
            session_data = {
                "pk": message.message["message"]["data"]["pk"],
                "role":"server",
                "node_id":message.message["node_id"],
                "node_type": message.message["node_type"],  
                "discovery_session_id": session_id   
            }
            self.make_function_call(self.sessions,"create_discovery_session",session_id,session_data)
        #prepare discovery response message
        msg_data ={
            "pk": EncryptionModule.format_public_key(self.pk),
            "discovery_session_id": session_id   
            }
        #print debug message
        #if self.DEBUG:
            #get all discovery sessions
        #    sessions = self.make_function_call(self.sessions,"get_discovery_sessions")
        #    loginfo(f"{self.node_id}:respond_to_discovery: attemping to connect to {message.message['node_id']}, discovery sessions : {sessions}")
        #send the message
        self.publisher.publish({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": msg_data,
                                      "type": "discovery_response",
                                      "signed":True})
    
    def verify_discovery(self,message):
        #verify discovery request and send challenge response
        #first verify the message     
        try :
            message=DiscoveryResponseMessage(message)
        except Exception as e:
            if self.DEBUG:
                loginfo(f"{self.node_id}: error validating message : {e}")
            return None
        #check if the node has active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session_by_node_id",message.message["node_id"])
        if session:
            if session.get("challenge"):
                if self.DEBUG:    
                    loginfo(f"{self.node_id}: discovery session is already active")
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
            "node_id":message.message["node_id"],
            "node_type": message.message["node_type"],
            "challenge": challenge,
            "client_challenge_response": client_sol,
            "server_challenge_response": server_sol,
            "discovery_session_id": message.message["message"]["data"]["discovery_session_id"]
        }
        #create discovery session
        self.make_function_call(self.sessions,"create_discovery_session",message.message["message"]["data"]["discovery_session_id"],session_data)
        #prepare verification message 
        msg_data = {
            "challenge": challenge,
            "client_challenge_response": client_sol,
            "discovery_session_id": message.message["message"]["data"]["discovery_session_id"]
            }
        #print debug message
        #if self.DEBUG:
        #    #get all discovery sessions
        #    sessions = self.make_function_call(self.sessions,"get_discovery_sessions")
        #    loginfo(f"{self.node_id}:verify_discovery: attemping to connect to {message.message['node_id']}, discovery sessions : {sessions}")
        #send the message
        self.publisher.publish({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": msg_data,
                                      "type": "discovery_verification",
                                      "signed":True})
 
    def verify_discovery_response(self,message):
        #verify discovery response and add node to the network
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session",message["message"]["data"]["discovery_session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #verify the message s
        try :
            message=VerificationMessage(message)
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
        self.make_function_call(self.sessions,"update_discovery_session",message.message["message"]["data"]["discovery_session_id"],session_data)
        #prepare verification message
        msg_data = {
            "challenge": challenge,
            "server_challenge_response": server_sol,
            "discovery_session_id": message.message["message"]["data"]["discovery_session_id"]
            }
        #print debug message
        #if self.DEBUG:
        #    #get all discovery sessions
        #    sessions = self.make_function_call(self.sessions,"get_discovery_sessions")
        #    loginfo(f"{self.node_id}:verify_discovery_response: attemping to connect to {message.message['node_id']}, discovery sessions : {sessions}")
        #send the message
        self.publisher.publish({
            "target": message.message["node_id"],
            "time":mktime(datetime.datetime.now().timetuple()),
            "message": msg_data,
            "type": "discovery_verification_response",
            "signed":True})

    def approve_discovery(self,message):
        #approve discovery request and send approval response
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session",message["message"]["data"]["discovery_session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #verify the message
        try :
            message=VerificationResponseMessage(message)
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
        #session_id = self.generate_session_id()
        session_id = message.message["message"]["data"]["discovery_session_id"]
        #create new session
        session_data = {
            "pk": session["pk"],
            "node_id": message.message["node_id"],
            "node_type": message.message["node_type"],
            "last_active": mktime(datetime.datetime.now().timetuple()),
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
            "test_message": EncryptionModule.encrypt_symmetric("client_test",key),
            "discovery_session_id":message.message["message"]["data"]["discovery_session_id"]
            }
        #print debug message
        #if self.DEBUG:
            #get all discovery sessions
        #    sessions = self.make_function_call(self.sessions,"get_discovery_sessions")
            #get all connection sessions 
        #    connection_sessions = self.make_function_call(self.sessions,"get_connection_sessions")
        #    loginfo(f"{self.node_id}:approve_discovery: attemping to connect to {message.message['node_id']}, discovery sessions : {sessions}, connection sessions : {connection_sessions}")
        #send the message
        self.publisher.publish({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": msg_data,
                                      "type": "discovery_approval",
                                      "signed":True})
            
    def approve_discovery_response(self,message):
        #approve discovery response and add node to the network
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_discovery_session",message["message"]["data"]["discovery_session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #validate the message
        try :
            message=ApprovalMessage(message)
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
        #prepare approval message
        msg_data = {
            "session_id": session_id,
            "test_message": EncryptionModule.encrypt_symmetric("server_test",key),
            "discovery_session_id":message.message["message"]["data"]["discovery_session_id"]
            }
        #send the message
        self.publisher.publish({
            "target": message.message["node_id"],
            "time":mktime(datetime.datetime.now().timetuple()),
            "message": msg_data,
            "type": "discovery_approval_response",
            "signed":True})
        #delay for 1 second
        sleep(1)
        self.make_function_call(self.sessions,"create_connection_session",session_id,session_data)
        #print debug message
        #if self.DEBUG:
            #get all discovery sessions
        #    sessions = self.make_function_call(self.sessions,"get_discovery_sessions")
            #get all connection sessions 
        #    connection_sessions = self.make_function_call(self.sessions,"get_connection_sessions")
        #    loginfo(f"{self.node_id}:approve_discovery_response: attemping to connect to {message.message['node_id']}, discovery sessions : {sessions}, connection sessions : {connection_sessions}")
        loginfo(f"{self.node_id}: Discovery completed successfully with {message.message['node_id']}")

    def finalize_discovery(self,message):
        #approve discovery response and add node to the network
        #check if the node does not have active discovery session with the sender
        session = self.make_function_call(self.sessions,"get_connection_session",message["message"]["data"]["discovery_session_id"])
        if not session:
            if self.DEBUG:
                loginfo(f"{self.node_id}: node does not have active discovery session with the sender")
            return None
        #validate the message
        try :
            message=ApprovalResponseMessage(message)
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
        #print debug message
        #if self.DEBUG:
            #get all discovery sessions
        #    sessions = self.make_function_call(self.sessions,"get_discovery_sessions")
            #get all connection sessions 
        #    connection_sessions = self.make_function_call(self.sessions,"get_connection_sessions")
        #    loginfo(f"{self.node_id}:finalize_discovery: attemping to connect to {message.message['node_id']}, discovery sessions : {sessions}, connection sessions : {connection_sessions}")
        loginfo(f"{self.node_id}: Discovery completed successfully with {message.message['node_id']}")
        
if __name__ == '__main__':
    ns = get_namespace()
    
    try :
        node_id= get_param(f'{ns}discovery/node_id') # node_name/argsname
        loginfo(f"discovery: Getting node_id argument, and got : {node_id}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")
    
    try :
        node_type= get_param(f'{ns}discovery/node_type') # node_name/argsname
        loginfo(f"discovery: Getting endpoint argument, and got : {node_type}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_type")
    
    try :
        secret= get_param(f'{ns}discovery/secret') # node_name/argsname
        loginfo(f"discovery: Getting secret argument, and got : {secret}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : secret")
    
    try:
        max_delay = get_param(f'{ns}discovery/max_delay',10)
        loginfo(f"discovery: Getting max_delay argument, and got : {max_delay}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : max_delay")
    
    node = DiscoveryProtocol(node_id,node_type,secret,max_delay,DEBUG=True)
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