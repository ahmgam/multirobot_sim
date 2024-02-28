'''
This is Point-to-Point Protocol (P2P) message format for the network peer discovery and data transfer.

Network peer discovery message format:
{
    
    "type": "discovery",
    "node_id": "node id"
    "node_type": "node type"
    "port": "port"
    "pos": "position"
    "session_id": "[discovery/data]-session id"
    "message":{
        "timestamp": "timestamp"
        "counter": "counter"
        data: {
            "pk": "public key"
        }
    }
    "hash": "hash"
    "signature": "signature"
}

Network peer discovery response message format:
{
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "type": "discovery_response",
    "session_id": "[discovery/data]-session id"
*   "message":{
        "timestamp": "timestamp"
        "counter": "counter"
        "data":{
            "pk": "public key"
        }
    }
    "hash": "hash"
    "signature": "signature"
}

Network peer discovery verification message format:
{
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "type": "discovery_verification",
    "session_id": "[discovery/data]-session id"
*   "message": {
        "timestamp": "timestamp"
        "counter": "counter"
        "data":{
            "challenge": "challenge"
            "client_challenge_response": "challenge_response"
        }
        
    }
    "hash": "hash"
    "signature": "signature"
}


Network Peer discovery verification response message format:
{
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "type": "discovery_verification_response",
    "session_id": "[discovery/data]-session id"
    "message: {
        "timestamp": "timestamp"
        "counter": "counter"
        "data":{
            "challenge": "challenge"
            "server_challenge_response": "challenge_response"
            }
    }
    "hash": "hash"
    "signature": "signature"
    
}
Network peer approval message format:
{
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "type": "discovery_approval",
    "session_id": "[discovery/data]-session id"
    "message: {
        "timestamp": "timestamp"
        "counter": "counter"
        "data":{
            "session_id": "session id"
            "session_key": "session key"
            "test_message": "test message"
            }
    }
    "hash": "hash"
    "signature": "signature"
    
}
Network peer approval response message format:
{
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "type": "approval_response",
    "session_id": "[discovery/data]-session id"
    "message: {
        "timestamp": "timestamp"
        "counter": "counter"
        "data":{
            "session_id": "session key"
            "test_response": "test response"
            }
    }
    "hash": "hash"
    "signature": "signature"
}

Network peer heartbeat message format:
{
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "session_id": "[discovery/data]-session id"
    "type": "heartbeat",
    "message":{
        "timestamp": "timestamp"
    }
    
    
}

Network peer heartbeat response message format:
{
    "session_id": "[discovery/data]-session id"
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "type": "heartbeat_response",
    "message":{
        "timestamp": "timestamp"
        "counter": "counter"
    }
}

Network peer data message format:
{
 
    "session_id": "[discovery/data]-session id"
    "node_id": "node id",
    "node_type": "node type"
    "pos": "position"
    "port": "port"
    "type": "data_message",
    "message":{
        "timestamp": "timestamp"
        "counter": "counter"
        "data": {}
    }
}


'''

import json
from collections import OrderedDict
from rospy import loginfo
from multirobot_sim.msg import KeyValueArray
from rosmsg import KeyValue
#########################################
# Messages 
#########################################

class Message :
    def __init__(self,data, **kwargs):
        #validate the message
        self.__validate(data)
        #save the message
        self.message =OrderedDict( data)
        #check if required fields are present
        if "required_fields" in kwargs:
            self.__check_required_fields(kwargs["required_fields"],data)
        
            
    def __check_required_fields(self,required_fields,data):
        if not isinstance(required_fields,list):
            loginfo("required_fields must be a list")
            raise TypeError("required_fields must be a list")    
        for value in required_fields:
            if value not in data["message"]["data"].keys():
                loginfo("field {} is required".format(value))
                raise ValueError("field {} is required".format(value))

    def __validate(self,data):
        #validate the message
        #if not (isinstance(data,dict) or isinstance(data,OrderedDict)):
        #    raise TypeError("data must be a dictionary")
        for key in ["type","session_id","node_id","message","node_type","time"]:
            if key not in data:
                raise ValueError("field {} is required".format(key))

            
    def __repr__(self):
        return json.dumps(self.message)
    
    def __str__(self):
        return json.dumps(self.message)

    def to_dict(self):
        return dict(self.message)
    
    def to_json(self):
        return json.dumps(self.message)
        
    def sign(self, private_key):
        #sign the message
        self.message["signature"] = private_key.sign(self.message["hash"].encode("utf-8"))
        
class DiscoveryMessage(Message):
    def __init__(self,data):
        #definte required fields
        required_fields = ["pk"]
        super().__init__(data,required_fields=required_fields)
        
class DiscoveryResponseMessage(Message):
    def __init__(self,data):
        #definte required fields
        required_fields = ["pk"]
        super().__init__(data,required_fields=required_fields)  
              
class VerificationMessage(Message):
    def __init__(self,data):
        #definte required fields
        required_fields = ["challenge","client_challenge_response"]
        super().__init__(data,required_fields=required_fields)
   
class VerificationResponseMessage(Message):
    def __init__(self,data):
        #definte required fields
        required_fields = ["challenge","server_challenge_response"]
        super().__init__(data,required_fields=required_fields)   
        
class ApprovalMessage(Message):
    def __init__(self,data):
        #definte required fields
        required_fields = ["session_id","session_key","test_message"]
        super().__init__(data,required_fields=required_fields) 
        
class ApprovalResponseMessage(Message):
    def __init__(self,data):
        #definte required fields
        required_fields = ["session_id","test_message"]
        super().__init__(data,required_fields=required_fields)
 
 
def unflatten_json(flat_json, sep='.'):
    unflattened_json = {}
    for compound_key, value in flat_json.items():
        keys = compound_key.split(sep)
        current_level = unflattened_json
        for key in keys[:-1]:  
            if key not in current_level:
                current_level[key] = {}
            current_level = current_level[key]
        current_level[keys[-1]] = value  
    return unflattened_json

def flatten_json(nested_json, parent_key='', sep='.'):
    items = []
    for key, value in nested_json.items():
        new_key = f"{parent_key}{sep}{key}" if parent_key else key
        if isinstance(value, dict):
            items.extend(flatten_json(value, new_key, sep=sep).items())
        else:
            items.append((new_key, value))
    return dict(items)


def pack_key_value_array(src_dict):
    key_value_array = KeyValueArray()
    kv_list = []
    for key, value in src_dict.items():
        key_value = KeyValue()
        key_value.key = key
        key_value.value = value
        kv_list.append(key_value)
    key_value_array.items = kv_list
    return key_value_array

def unpack_key_value_array(key_value_array):
    dst_dict = {}
    for kv in key_value_array.items:
        dst_dict[kv.key] = kv.value
    return dst_dict
        