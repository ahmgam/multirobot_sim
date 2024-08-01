#!/usr/bin/env python
from collections import OrderedDict
from time import mktime
from datetime import datetime
from random import choices
from string import ascii_uppercase, digits
from multirobot_sim.srv import FunctionCall, FunctionCallResponse
from encryption import EncryptionModule
import json
import rospy
import traceback

class SessionManager:
    def __init__(self,node_id):
        #define session manager
        self.node_id = node_id
        self.discovery_sessions =OrderedDict()
        self.connection_sessions = OrderedDict()
        self.node = rospy.init_node("session_manager", anonymous=True)
        
        #define key store proxy
        rospy.loginfo(f"{self.node_id}: Discovery:Initializing key store service")
        self.key_store = rospy.ServiceProxy(f"/{self.node_id}/key_store/call", FunctionCall)
        self.key_store.wait_for_service(timeout=100)
        #get public and private key 
        keys  = self.make_function_call(self.key_store,"get_rsa_key")
        self.pk = keys["pk"]
        self.sk = keys["sk"]
        self.node_states = OrderedDict({self.node_id:{"pk":self.pk,"last_active":mktime(datetime.now().timetuple())}})
        self.refresh_node_state_table()
        rospy.loginfo(f"{self.node_id}: SessionManager:Initializing sessions service")
        self.server = rospy.Service(f"/{self.node_id}/sessions/call", FunctionCall, self.handle_function_call)
        rospy.loginfo(f"{self.node_id}: SessionManager:Initialized successfully")
        
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
        try:
            if hasattr(self,function_name):
                if len(args) == 0:
                    response = getattr(self,function_name)()
                else:
                    response = getattr(self,function_name)(*args)
            else:
                response = None
        except Exception as e:
            print(f"error in sessions : {traceback.print_tb(e.__traceback__)}")
            response = None
        if response == None:
            response = FunctionCallResponse(r'{}')
        else:
            response = json.dumps(response) if type(response) is not str else response
            response = FunctionCallResponse(response)
        
        return response
        
    def create_discovery_session(self,session_id, data):
        #create new session with the given public key and type
        #add last call timestamp
        data["last_active"] = mktime(datetime.now().timetuple())
        self.discovery_sessions[session_id]= data
        
    def get_discovery_sessions(self):
        #get all discovery sessions
        return dict(self.discovery_sessions)
            
    def update_discovery_session(self,session_id, data):
        #update session with the given public key and type
        for key,value in data.items():
            self.discovery_sessions[session_id][key] = value
        #update last call timestamp
        self.discovery_sessions[session_id]["last_active"] = mktime(datetime.now().timetuple())
        
    def get_discovery_session(self, session_id):
        #get all discovery sessions
        session = self.discovery_sessions.get(session_id,None)
        if session:
            #update last call timestamp
            self.discovery_sessions[session_id]["last_active"] = mktime(datetime.now().timetuple()) 
        return session
    
    def get_discovery_session_by_node_id(self, node_id):
        #get all discovery sessions
        session = None
        for discovery_session in self.discovery_sessions.values():
            if discovery_session["node_id"] == node_id:
                session= discovery_session
        #if session:
            #update last call timestamp
            #self.discovery_sessions[session["session_id"]]["last_active"] = mktime(datetime.now().timetuple()) 
        return session
    def has_active_connection_session(self, node_id):
        #check if session with the given public key is active
        for key,value in self.connection_sessions.items():
            if value["node_id"] == node_id:
                return True
        return False
    
    def get_connection_session(self,session_id):
        #get connection sessions
        return self.connection_sessions.get(session_id,None)
    
    def get_connection_sessions(self):
        return dict(self.connection_sessions)
        
    def create_connection_session(self, session_id, data):
        #create new session with the given public key and type
        self.connection_sessions[session_id]= data
        #refresh node state table
        self.refresh_node_state_table()
        
    def update_connection_session(self, session_id, data):
        #update session with the given public key and type
        for key,value in data.items():
            self.connection_sessions[session_id][key] = value
        #update last call timestamp
        self.connection_sessions[session_id]["last_active"] = mktime(datetime.now().timetuple())
    
    def get_connection_session_by_node_id(self, node_id):
        #get connection session by node id
        for key,value in self.connection_sessions.items():
            if value["node_id"] == node_id:
                return value
        return None
        
    def get_active_nodes(self):
        ret_data = []
        for session in self.connection_sessions.values():
            if session["last_active"] > mktime(datetime.now().timetuple())-60 and session["status"] == "active":
                ret_data.append(session["node_id"])
        return ret_data

    def is_conntected(self):
        if len(self.get_active_nodes()) > 0:
            return True
        return False
    
    def get_active_nodes_with_pk(self):
        return [{session["node_id"]:session["pk"]} for session in self.connection_sessions.values() if session["last_active"] > mktime(datetime.now().timetuple())-60]
    
    def get_node_state_table(self):
        #refresh node state table
        self.refresh_node_state_table()
        #get nodes in state table
        response = {}
        for key,value in self.node_states.items():
            if value["last_active"] > mktime(datetime.now().timetuple())-60:
                response[key] = value["pk"]
        return response
    
    
    def update_node_state_table(self,table):
        #refresh node state table
        self.refresh_node_state_table()
        #update node state table
        for key,value in table.items():
            #check if node is already in node state table
            if key in self.node_states.keys():
                #update last call timestamp
                self.node_states[key]["last_active"] = mktime(datetime.now().timetuple())
                continue
            #update last call timestamp
            self.node_states[key] = {"pk":value,"last_active":mktime(datetime.now().timetuple())}
            
    def compare_node_state_table(self,table):
        #refresh node state table
        self.refresh_node_state_table()
        #compare node state table
        for key,value in table.items():
            #check if node is already in node state table
            if key in self.node_states.keys():
                if self.node_states[key]["pk"] == value:
                    continue
                else:
                    return False
        return True
    
    def refresh_node_state_table(self):
        #refresh node state table
        for key,value in self.connection_sessions.items():
            if value["node_id"] in self.node_states.keys():
                continue
            else:
                self.node_states[value["node_id"]] = {"pk":value["pk"],"last_active":mktime(datetime.now().timetuple())}
                
    def get_connection_sessions(self):
        return {key:value for key,value in  self.connection_sessions.items() if value["status"] == "active"}
    
    def get_node_state(self,node_id):
        if node_id in self.node_states.keys():
            return self.node_states[node_id]
        else:
            return None

if __name__ == "__main__":
    ns = rospy.get_namespace()
    try :
        node_id= rospy.get_param(f'{ns}roschain/node_id') # node_name/argsname
    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_id")
    session = SessionManager(node_id)
    rospy.spin()