from random import choices
from string import ascii_lowercase
import json
from encryption import EncryptionModule
from math import ceil
from time import mktime
import datetime
from rospy import loginfo,ServiceProxy,Publisher,Subscriber,init_node,get_namespace,get_param,ROSInterruptException
from multirobot_sim.srv import FunctionCall
from std_msgs.msg import String
from queue import Queue

#######################################
# Consensus protocol SPFT
#######################################

class SBFT:
    def __init__(self,node_id,node_type,DEBUG=False) -> None:
        #define node id
        self.node_id = node_id
        #define node type
        self.node_type = node_type
        #define debug mode
        self.DEBUG = DEBUG
        #define views
        self.views = {}
        #define view timeout
        self.view_timeout = 10
        #define node 
        self.node = init_node("consensus",anonymous=True)
        #init sessions
        self.sessions = ServiceProxy("sessions/call",FunctionCall)
        #init network 
        self.network = ServiceProxy("network/call",FunctionCall)
        #init blockchain
        self.blockchain = ServiceProxy("blockchain/call",FunctionCall)
        #message publisher
        self.publisher = Publisher("send_message",String,queue_size=10)
        #message subscriber 
        self.subscriber = Subscriber("consensus_handler",String,self.handle_message)
        #define key store proxy
        self.key_store = ServiceProxy('key_store/call', FunctionCall)
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk = keys["pk"]
        self.sk = keys["sk"]
        # queue
        self.queue = Queue()
        
    def cron(self):
        #TODO implement cron for view timeout
        #check views for timeout
        for view_id,view in self.views.copy().items():
            if mktime(datetime.datetime.now().timetuple()) - view['last_updated'] > self.view_timeout:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: View {view_id} timed out")
                self.views.pop(view_id)

    def make_function_call(self,service,function_name,*args):
        args = json.dumps(args)
        response = service(function_name,args).response
        if response == r"{}":
            return None
        return json.loads(response)
    
    def handle_message(self,msg):
        #parese message as json
        msg = json.loads(msg.data)
        #push message to queue
        self.queue.put(msg)
        
    def handle(self, msg):
        #handle message
        msg = json.loads(msg.data)
        msg = msg["message"]["data"]
        operation = msg['operation']
        if operation == 'pre-prepare':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting pre-prepare")
            self.pre_prepare(msg)
        elif operation == 'prepare':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting prepare")
            self.prepare(msg)
        elif operation == 'prepare-collect':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting prepare-collect")
            self.prepare_collect(msg)
        elif operation == 'commit':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting commit")
            self.commit(msg)
        elif operation == 'commit-collect':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting commit-collect")
            self.commit_collect(msg)
        elif operation == 'sync_request':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting sync_request")
            self.make_function_call(self.blockchain,"handle_sync_request",msg)
        elif operation == 'sync_reply':
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting sync_response")
            self.make_function_call(self.blockchain,"handle_sync_reply",msg)
        else:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Received message from {msg['message']['node_id']} of type {msg['message']['type']}, but no handler found")
            pass
    
    def send(self,msg):
        #check message type 
        if not type(msg['message']) in [dict,str]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Invalid message type")
            return
        #create view number 
        view_id = self.generate_view_id()
        #get node_ids 
        node_ids = self.make_function_call(self.sessions,"get_node_state_table")
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
            "hash": EncryptionModule.hash(json.dumps(msg['message'])),
            "node_ids":node_ids
            }
        #add data to message
        msg["operation"]="pre-prepare"
        msg['view_id'] = view_id
        msg["node_ids"] = node_ids
        #serialize message
        msg_data = json.dumps(msg)
        #sign message
        msg_signature = EncryptionModule.sign(msg_data,self.sk)
        #add signature to message
        msg["signature"] = msg_signature
        
        #broadcast message to the network
        self.make_function_call(self.network,"send_message",'all',msg)
    
    def pre_prepare(self,msg):
        #handle pre-prepare message
        #check if view exists
        view_id = msg['view_id']
        if view_id in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is already created")
            return
        #get the session 
        session = self.make_function_call(self.sessions,"get_connection_session_by_node_id",msg['source'])
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)        
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified")
            return None
        #compare node state table
        if not self.make_function_call(self.sessions,"compare_node_state_table",msg['node_ids']):
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node state table not equal")
            return None
        #message payload
        payload = {
            "timestamp":mktime(datetime.datetime.now().timetuple()),
            "operation":"prepare",
            "source":self.node_id,
            "view_id":view_id,
            "message":msg['message']
        }
        #get hash and sign of message
        msg_data = json.dumps(payload)
        msg_signature = EncryptionModule.sign(msg_data,self.sk)
        #add signature to message
        payload["hash"]=EncryptionModule.hash(json.dumps(msg["message"]))
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
            "node_ids":msg['node_ids']
        }
        #send_message
        self.make_function_call(self.network,"send_message",msg['source'],payload)
    
    def prepare(self,msg):
        #handle prepare message
        #check if view exists
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #loginfo(view)
        #get session
        session = self.make_function_call(self.sessions,"get_connection_session_by_node_id",msg['source'])
        #check if node_id is not the source
        #loginfo(session)
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        msg_hash = msg.pop('hash')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified")
            return None
        #check hash of message
        if msg_hash != view["hash"]:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Hash of message does not match")
            return None
        msg["signature"] = msg_signature
        msg["hash"] = msg_hash
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
            "prepare":self.views[view_id]["prepare"],
            "hash":view["hash"]
        }
        #get hash and sign of message
        msg_data = json.dumps(payload)
        msg_hash = EncryptionModule.hash(msg_data)
        msg_signature = EncryptionModule.sign_hash(msg_hash,self.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "prepare"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        #broadcast message
        self.make_function_call(self.network,"send_message",'all',payload)
        
    
    def prepare_collect(self,msg):
        #handle prepare-collect message
        #check if view exists
        #loginfo(msg)
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #get session
        session = self.make_function_call(self.sessions,"get_connection_session_by_node_id",msg['source'])
        #check if node_id is not the source
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: message signature not verified")
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
        for m in msg["prepare"]:
            #verify signature
            m_signature = m.pop('signature')
            m_hash = m.pop('hash')
            m_data = json.dumps(m)
            #verify the message signature
            node_state = self.make_function_call(self.sessions,"get_node_state",m['source'])
            if EncryptionModule.verify(m_data, m_signature, EncryptionModule.reformat_public_key(node_state["pk"])) == False:
                if self.DEBUG:
                    loginfo(f"{self.node_id}: signature of {m['source']} not verified")
                return None
            #check hash of message
            if m_hash != view["hash"]:
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
        msg_data = json.dumps(payload)
        msg_hash = EncryptionModule.hash(msg_data)
        msg_signature = EncryptionModule.sign_hash(msg_hash,self.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "commit"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        self.make_function_call(self.network,"send_message",view["source"],payload)
    def commit(self,msg):
        #handle commit message
        #check if view exists
        #loginfo(msg)
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #get session
        session = self.make_function_call(self.sessions,"get_connection_session_by_node_id",msg['source'])
        #check if node_id is not the source
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified")
            return None
        #check hash of message
        if msg["hash"]  != view["hash"]:
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
        msg_data = json.dumps(payload)
        msg_hash = EncryptionModule.hash(msg_data)
        msg_signature = EncryptionModule.sign_hash(msg_hash,self.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "complete"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        #push message to output queue
        try:
            self.parent.queues.put_output_queue(view["message"],view["source"],"dict",view["timestamp"])
        except Exception as e:
            print(f"{self.node_id}: ERROR : {e}")
        #broadcast message
        self.make_function_call(self.network,"send_message",'all',payload)
    
    def commit_collect(self,msg):
        #handle commit-collect message
        #check if view exists
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.DEBUG:
                loginfo(f"{self.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #get session
        session = self.make_function_call(self.sessions,"get_connection_session_by_node_id",msg['source'])
        #check if node_id is not the source
        if self.node_id == msg['source']:
            if self.DEBUG:
                loginfo(f"{self.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.DEBUG:
                loginfo(f"{self.node_id}: signature not verified")
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
        self.parent.queues.put_output_queue(view["message"],view["source"],"dict",view["timestamp"])
    
    #TODO implement view change
    def generate_view_id(self,length=8):
        #generate view id
        return ''.join(choices(ascii_lowercase, k=length))
    