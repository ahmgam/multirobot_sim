#!/usr/bin/env python
from random import choices
from string import ascii_lowercase
import json
import pickle
from encryption import EncryptionModule
from math import ceil
from time import mktime,time
import datetime
from rospy import ServiceProxy,Publisher,Subscriber,Service,ROSInterruptException,Rate,init_node,get_namespace,get_param,loginfo,is_shutdown
from multirobot_sim.srv import FunctionCall,FunctionCallResponse
from std_msgs.msg import String
from std_srvs.srv import Trigger,TriggerResponse
from queue import Queue
from messages import MessagePublisher,MessageSubscriber
#######################################
# Consensus protocol SPFT
#######################################

class SBFT:
    def __init__(self,node_id,node_type,timeout_interval,DEBUG=False) -> None:
        #define node id
        self.node_id = node_id
        #define node type
        self.node_type = node_type
        #define debug mode
        self.DEBUG = DEBUG
        #define views
        self.views = {}
        #define view timeout
        self.view_timeout = timeout_interval
        #define node 
        self.node = init_node("consensus",anonymous=True)
        #define function call service
        #define key store proxy
        loginfo(f"{self.node_id}: SBFT:Initializing key store service")
        self.key_store = ServiceProxy(f"/{self.node_id}/key_store/call", FunctionCall)
        self.key_store.wait_for_service(timeout=100)
        #get public and private key 
        self.keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk,self.sk =EncryptionModule.reconstruct_keys(self.keys["pk"],self.keys["sk"])
        #init sessions
        loginfo(f"{self.node_id}: SBFT:Initializing sessions service")
        self.sessions = ServiceProxy(f"/{self.node_id}/sessions/call",FunctionCall,True)
        self.sessions.wait_for_service(timeout=100)
        #init blockchain
        loginfo(f"{self.node_id}: SBFT:Initializing blockchain service")
        self.blockchain = ServiceProxy(f"/{self.node_id}/blockchain/call",FunctionCall,True)
        self.blockchain.wait_for_service(timeout=100)
        #init prepare message 
        loginfo(f"{self.node_id}: SBFT:Initializing publisher and subscriber")
        self.prepare_message = MessagePublisher(f"/{self.node_id}/network/prepare_message")
        #message subscriber 
        self.subscriber = MessageSubscriber(f"/{self.node_id}/consensus/consensus_handler",self.handle_message)
        #define blockchain publisher 
        self.blockchain_publisher = MessagePublisher(f"/{self.node_id}/blockchain/blockchain_handler")
        # queue
        self.queue = Queue()
        #input queue
        self.input_queue = Queue()
        #completed views
        self.completed_views = []
        #ongoinng view
        self.ongoing_view = None
        #failed queue, handling it is in TODO list
        self.failed_queue = Queue()
        loginfo(f"{self.node_id}: SBFT:Initializing function call service")
        self.server = Service(f"/{self.node_id}/consensus/call",FunctionCall,self.handle_function_call)
        #define is_idle service
        self.is_idle_service = Service(f"/{self.node_id}/consensus/is_idle",Trigger,self.is_idle)
        #log publisher
        self.log_publisher = Publisher(f"/{self.node_id}/connector/send_log", String, queue_size=10)
        loginfo(f"{self.node_id}: SBFT:Initialized successfully")
        
    def cron(self):
        #TODO implement cron for view timeout
        #check views for timeout
        for view_id,view in self.views.copy().items():
            if mktime(datetime.datetime.now().timetuple()) - view['last_updated'] > self.view_timeout:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: View {view_id} timed out")
                self.views.pop(view_id)
                #check if ongoing view is the same as view_id
                if self.ongoing_view == view_id:
                    #put view in failed queue
                    self.failed_queue.put(view)
                    loginfo(f"{self.node_id}: View {view_id} added to failed queue")
                    self.log_publisher.publish(f"{mktime(datetime.datetime.now().timetuple())},failed,{view['message']['msg_id']},{view['timestamp']}")
                    self.ongoing_view = None

    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    
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
    
    def handle_message(self,msg):
        #parese message as json
        #msg = json.loads(msg.data)
        #push message to queue
        #self.queue.put(msg)
        self.queue.put(msg)
    def handle(self, msg):
        #handle message
        msg, session = msg["message"], msg.get("session")
        error = False
        try:
            msg= msg["data"]
        except : 
            print(f"error with message with type {type(msg)}: {msg} and session : {session}")
            error = True
            
        if error:
            try:
                msg = pickle.loads(msg)
                msg= msg["data"]
            except:
                print(f"error with message with type {type(msg)}: {msg} and session : {session}")
                exit()
            
        operation = msg['operation']
        #start_time = time()
        if operation == 'submit':
            
            self.input_queue.put(msg)
            #self.send(msg)
            #print(f"Time taken for pre_prepare: {time()-start_time}")
        elif operation == 'pre-prepare':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']} and {msg['view_id']}, starting pre-prepare")
            self.pre_prepare(msg,session)
            #print(f"Time taken for pre_prepare: {time()-start_time}")
        elif operation == 'prepare':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']} and {msg['view_id']}, starting prepare")
            self.prepare(msg,session)
            #print(f"Time taken for prepare: {time()-start_time}")
        elif operation == 'prepare-collect':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']} and {msg['view_id']}, starting prepare-collect")
            self.prepare_collect(msg,session)
            #print(f"Time taken for prepare_collect: {time()-start_time}")
        elif operation == 'commit':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']} and {msg['view_id']}, starting commit")
            self.commit(msg,session)
            #print(f"Time taken for commit: {time()-start_time}")
        elif operation == 'commit-collect':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']} and {msg['view_id']}, starting commit-collect")
            self.commit_collect(msg,session)
            #print(f"Time taken for commit_collect: {time()-start_time}")
        elif operation == 'sync_request':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']} and {msg['view_id']}, starting sync_request")
            self.make_function_call(self.blockchain,"handle_sync_request",msg)
            #print(f"Time taken for sync_request: {time()-start_time}")
        elif operation == 'sync_reply':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']} and {msg['view_id']}, starting sync_response")
            self.make_function_call(self.blockchain,"handle_sync_reply",msg)
            #print(f"Time taken for sync_reply: {time()-start_time}")
        else:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['message']['node_id']} of type {msg['message']['type']}, but no handler found")
            pass
    
    def is_idle(self,msg):
        #check if the node is idle
        if self.ongoing_view:
            return TriggerResponse(False,"Not Ready")
        return TriggerResponse(True,"Ready")
    
    def send(self,msg):
        if self.DEBUG:
            loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting send")
        #check message type 
        if not type(msg['message']) in [dict,str]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid message type : {type(msg['message'])}")
            return
        #create view number 
        view_id = self.generate_view_id()
        #set as ongoing view
        self.ongoing_view = view_id
        #get node_ids 
        node_ids = self.make_function_call(self.sessions,"get_node_state_table")
        #start time
        start_time = mktime(datetime.datetime.now().timetuple())
        #create view
        self.views[view_id] = {
            "timestamp":msg['timestamp'],
            "last_updated":mktime(datetime.datetime.now().timetuple()),
            "source": self.node_id,
            "message":msg['message'],
            "prepare":[],
            "commit":[],
            "view_id":view_id,
            "status":"prepare",
            "hash": EncryptionModule.hash(msg['message']),
            "node_ids":node_ids,
            "start_time":start_time
            }
        #add data to message
        msg["operation"]="pre-prepare"
        msg['view_id'] = view_id
        msg["node_ids"] = node_ids
        msg["start_time"] = start_time
        #sign message
        msg_payload = json.dumps(msg)
        msg_signature = EncryptionModule.sign(msg_payload,self.sk)
        #add signature to message
        msg["signature"] = msg_signature
        #broadcast message to the network
        self.prepare_message.publish({"message":msg,"type":"data_exchange","target":list(node_ids.keys())})
    
    def pre_prepare(self,msg,session):
        #handle pre-prepare message
        #check if view exists
        view_id = msg['view_id']
        if view_id in self.completed_views:
            if self.DEBUG:
                loginfo(f"{self.node_id}: View {view_id} is already completed")
            return
        if view_id in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is already created in pre-prepare")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        msg_payload = json.dumps(msg)      
        #verify the message signature
        if EncryptionModule.verify(msg_payload, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified in pre-prepare")
            return None
        #compare node state table
        #if not self.make_function_call(self.sessions,"compare_node_state_table",msg['node_ids']):
        #    if self.DEBUG:
        #        loginfo(f"{self.node_id}: Node state table not equal")
        #    return None
        #message payload
        payload = {
            "timestamp":mktime(datetime.datetime.now().timetuple()),
            "operation":"prepare",
            "source":self.node_id,
            "view_id":view_id,
            "message":msg['message'],
            "hash":EncryptionModule.hash(msg["message"])
        }
        #get hash and sign of message
        msg_payload = json.dumps(payload)
        msg_signature = EncryptionModule.sign(msg_payload,self.sk)
        #add signature to message
        payload["signature"]=msg_signature
        #create view
        self.views[view_id] = {
            "timestamp":msg['timestamp'],
            "last_updated":mktime(datetime.datetime.now().timetuple()),
            "source": msg['source'],
            "message":msg['message'],
            "prepare":[],
            "commit":[],
            "view_id":view_id,
            "status":"prepare",
            "hash": payload["hash"],
            "node_ids":msg['node_ids'],
            "start_time":msg['start_time']
        }
        #send_message
        self.prepare_message.publish({"message":payload,"type":"data_exchange","target":msg['source']})
    
    def prepare(self,msg,session):
        #handle prepare message
        #check if view exists
        view_id = msg['view_id']
        if view_id in self.completed_views:
            if self.DEBUG:
                loginfo(f"{self.node_id}: View {view_id} is already completed")
            return
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is not created in prepare")
            return
        #get view 
        view = self.views[view_id]
        #check if node_id is in the node_ids
        if self.node_id not in view["node_ids"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is not in the node_ids in prepare")
            return
        #loginfo(view)
        #check if node_id is not the source
        #loginfo(session)
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        msg_payload = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_payload, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified in prepare")
            return None
        #check hash of message
        if msg["hash"] != view["hash"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Hash of message does not match")
            return None
        msg["signature"] = msg_signature

        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(msg['node_ids']):
        #    if self.DEBUG:
        #        loginfo("Node state table not equal")
        #    return None
        #add message to prepare
        self.views[view_id]["prepare"].append(msg)
        #check if the number of prepare is more than 
        if len(self.views[view_id]["prepare"]) < ceil((2/3)*((len(view["node_ids"])-1)/3)):
            return None
        #send prepare-collect message to source node
        payload = {
            "timestamp":mktime(datetime.datetime.now().timetuple()),
            "operation":"prepare-collect",
            "view_id":view_id,
            "source":self.node_id,
            "prepare":{p["source"]:p for p in self.views[view_id]["prepare"]},
            "hash":view["hash"]
        }
        #get hash and sign of message
        msg_payload = json.dumps(payload)
        msg_signature = EncryptionModule.sign(msg_payload,self.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "prepare"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        #broadcast message
        self.prepare_message.publish({"message":payload,"type":"data_exchange","target":list(view["node_ids"].keys())})
        
    
    def prepare_collect(self,msg,session):
        #handle prepare-collect message
        #check if view exists
        #loginfo(msg)
        view_id = msg['view_id']
        if view_id in self.completed_views:
            if self.DEBUG:
                loginfo(f"{self.node_id}: View {view_id} is already completed")
            return
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is not created in prepare-collect")
            return
        #get view 
        view = self.views[view_id]
        #check if node_id is in the node_ids
        if self.node_id not in view["node_ids"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is not in the node_ids")
            return
        #check if node_id is not the source
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #msg_hash = msg.pop('hash')
        msg_payload = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_payload, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified in prepare-collect")
            return None
        #check hash of message
        if msg["hash"] != view["hash"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Hash of message does not match")
            return None
        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(msg['node_ids']):
        #    if self.DEBUG:
        #        loginfo("Node state table not equal")
        #    return None
        #loop in prepare-collect
        for m in msg["prepare"].values():
            #verify signature
            m_signature = m.pop('signature')
            msg_payload = json.dumps(m)
            #verify the message signature
            node_state = view["node_ids"].get(m["source"])
            if EncryptionModule.verify(msg_payload, m_signature, EncryptionModule.reformat_public_key(node_state)) == False:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: signature of {m['source']} not verified")
                return None
            #check hash of message
            if m["hash"] != view["hash"]:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: Hash of message does not match")
                return None
        #send commit message to source node
        payload = {
            "timestamp":mktime(datetime.datetime.now().timetuple()),
            "operation":"commit",
            "view_id":view_id,
            "source":self.node_id,
            "hash":view["hash"]
        }
        #get hash and sign of message    
        msg_payload = json.dumps(payload)    
        msg_signature = EncryptionModule.sign(msg_payload,self.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "commit"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        self.prepare_message.publish({"message":payload,"type":"data_exchange","target":view["source"]})
    def commit(self,msg,session):
        #handle commit message
        #check if view exists
        #loginfo(msg)
        view_id = msg['view_id']
        if view_id in self.completed_views:
            if self.DEBUG:
                loginfo(f"{self.node_id}: View {view_id} is already completed")
            return
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View {view_id} is not created in commit, available views are {self.views.keys()}")
            return
        #get view 
        view = self.views[view_id]
        #check if node_id is in the node_ids
        if self.node_id not in view["node_ids"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is not in the node_ids")
            return
        #check if node_id is not the source
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #msg_hash = msg.pop('hash')
        msg_payload = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_payload, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified in commit")
            return None
        #check hash of message
        if msg["hash"] != view["hash"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Hash of message does not match")
            return None
        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(view['node_ids']):
        #    if self.DEBUG:
        #        loginfo("Node state table not equal")
        #    return None
        #add message to prepare
        msg["signature"] = msg_signature
        self.views[view_id]["commit"].append(msg)
        #check if the number of prepare is more than 
        if len(self.views[view_id]["commit"]) < ceil((2/3)*((len(view["node_ids"])-1)/3)):
            return None
        #send prepare-collect message to source node
        payload = {
            "timestamp":mktime(datetime.datetime.now().timetuple()),
            "operation":"commit-collect",
            "view_id":view_id,
            "source":self.node_id,
            "commit":self.views[view_id]["commit"]
        }
        #get hash and sign of message
        msg_payload = json.dumps(payload)
        msg_signature = EncryptionModule.sign(msg_payload,self.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "complete"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        #push message to output queue
        try:
            self.blockchain_publisher.publish({
                "data":view["message"],
                "type":"blockchain_data",
                "source":view["source"],
                "format":"dict",
                "time":view["timestamp"],
                "hash":view["hash"],
                "start_time":view["start_time"]
                })
        except Exception as e:
            print(f"{self.node_id}: ERROR : {e}")
        #broadcast message
        self.prepare_message.publish({"message":payload,"type":"data_exchange","target":list(view["node_ids"].keys())})
        #remove view
        self.completed_views.append(view_id)
        self.views.pop(view_id)
        #check if ongoing view is the same as view_id
        if self.ongoing_view == view_id:
            self.ongoing_view = None
    
    def commit_collect(self,msg,session):
        #handle commit-collect message
        #check if view exists
        view_id = msg['view_id']
        if view_id in self.completed_views:
            if self.DEBUG:
                loginfo(f"{self.node_id}: View {view_id} is already completed")
            return
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is not created in commit-collect")
            return
        #get view 
        view = self.views[view_id]
        #check if node_id is in the node_ids
        if self.node_id not in view["node_ids"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is not in the node_ids")
            return
        #check if node_id is not the source
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        msg_payload = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_payload, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified in commit-collect")
            return None

        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(msg['node_ids']):
        #    if self.DEBUG:
        #        loginfo("Node state table not equal")
        #    return None
        #loop in prepare-collect
        for m in msg["commit"]:
            #verify signature
            m_signature = m.pop('signature')
            m_data = json.dumps(m)
            #verify the message signature
            if EncryptionModule.verify(m_data, m_signature, EncryptionModule.reformat_public_key(view["node_ids"][m["source"]])) == False:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: signature not verified")
                return None
            #check hash of message
            if m["hash"] != view["hash"]:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: Hash of message does not match")
                return None
        #update view
        self.views[view_id]["status"] = "complete"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        #push message to output queue
        self.blockchain_publisher.publish({
            "data":view["message"],
            "type":"blockchain_data",
            "source":view["source"],
            "format":"dict",
            "time":view["timestamp"],
            "hash":view["hash"],
            "start_time":view["start_time"]
            })
        #remove view
        self.views.pop(view_id)
        self.completed_views.append(view_id)
        #check if ongoing view is the same as view_id
        if self.ongoing_view == view_id:
            self.ongoing_view = None
    
    #TODO implement view change
    def generate_view_id(self,length=8):
        #generate view id
        return ''.join(choices(ascii_lowercase, k=length))
    
if __name__ == "__main__":
    #get namespace 
    ns = get_namespace()
    try :
        node_id= get_param(f'{ns}consensus/node_id') # node_name/argsname
        loginfo(f"discovery: Getting node_id argument, and got : {node_id}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")
    
    try :
        node_type= get_param(f'{ns}consensus/node_type') # node_name/argsname
        loginfo(f"discovery: Getting node_type argument, and got : {node_type}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_type")
    
    try :
        timeout_interval= get_param(f'{ns}consensus/timeout_interval',10) # node_name/argsname
        loginfo(f"discovery: Getting timeout_interval argument, and got : {timeout_interval}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : timeout_interval")
    
    #define consensus
    consensus = SBFT(node_id,node_type,int(timeout_interval),False)
  
    rate = Rate(5)
    #start cron
    while not is_shutdown():
        consensus.cron()
        #check queue
        msg = consensus.queue.get()
        if msg:
            consensus.handle(msg)
        #check input queue
        if not consensus.ongoing_view:
            if not consensus.input_queue.empty():
                msg = consensus.input_queue.get()
                if msg:
                    consensus.send(msg)
        rate.sleep()
            