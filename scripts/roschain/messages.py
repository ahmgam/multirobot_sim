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
from multirobot_sim.msg import KeyValueArray,KeyValue
import re
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
          index= re.search('\[\d+\]', key)
          if index:
            key = key.replace(index.group(0),"")
            index = int(index.group(0)[1:-1])
            if key not in current_level:
                current_level[key] = [{}]
            if len(current_level[key]) == index:
              current_level[key].append(dict())
            current_level = current_level[key][index]
          else:
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
        elif isinstance(value,list):
          for i,item in enumerate(value):
            if isinstance(item, dict):
                items.extend(flatten_json(item, f"{new_key}[{i}]", sep=sep).items())
            else:
                items.append((f"{new_key}[{i}]", item))
        else:
            items.append((new_key, value))
    return dict(items)


def get_value_type(value):
    if isinstance(value,int):
        return "int"
    elif isinstance(value,float):
        return "float"
    elif isinstance(value,bool):
        return "bool"
    else:
        return "str"
    
def cast_value_type(value,type):
    if type == "int":
        if value in ["True", "False"]:
            return value == "True"
        return int(value)
    elif type == "float":
        return float(value)
    else:
        return str(value)
    
def pack_key_value_array(src_dict):
    key_value_array = KeyValueArray()
    #key_value_array = list()
    for key, value in src_dict.items():
        key_value = KeyValue()
        key_value.key = key
        key_value.value = str(value)
        key_value.type = get_value_type(value)
        key_value_array.items.append(key_value)
    return key_value_array

def unpack_key_value_array(key_value_array):
    dst_dict = {}
    for kv in key_value_array.items:
        dst_dict[kv.key] =  cast_value_type(kv.value,kv.type)
    return dst_dict
        

def dict_to_keyvaluearray(data):
    '''
    Prepare dictionary for message
    '''
    if not isinstance(data,dict):
        loginfo(f"data must be a dictionary, not {data}")
        raise TypeError(f"data must be a dictionary but got {data}")
    return pack_key_value_array(flatten_json(data))

def keyvaluearray_to_dict(kv_array):
    '''
    Prepare dictionary for message
    '''
    if not isinstance(kv_array,KeyValueArray):
        loginfo("data must be a KeyValueArray")
        raise TypeError("data must be a KeyValueArray")
    return unflatten_json(unpack_key_value_array(kv_array))


        
from rospy import Subscriber, Publisher
from std_msgs.msg import UInt16MultiArray,MultiArrayDimension
import pickle

#class MessagePublisher(Publisher):
#    def __init__(self, topic, *args, **kwargs):
#        super().__init__(topic,KeyValueArray, *args, **kwargs)
#
#    def publish(self, message):
#        message = dict_to_keyvaluearray(message)
#        super().publish(message)
        
#class MessageSubscriber(Subscriber):
#    def __init__(self, topic,callback, *args, **kwargs):
#        super().__init__(topic,KeyValueArray, lambda msg,*args:callback(keyvaluearray_to_dict(msg),*args),*args, **kwargs)
        
class MessagePublisher(Publisher):
    def __init__(self, topic, *args, **kwargs):
        super().__init__(topic,UInt16MultiArray, *args, **kwargs)

    def publish(self, message):
        message = list(pickle.dumps(message))
        ros_message = UInt16MultiArray()
        ros_message.data = message
        ros_message.layout.dim.append(MultiArrayDimension())  
        ros_message.layout.dim[0].size = len(message)  
        ros_message.layout.dim[0].stride = 1
        ros_message.layout.data_offset = 0  
        super().publish(ros_message)
        
class MessageSubscriber(Subscriber):
    def __init__(self, topic,callback, *args, **kwargs):
        super().__init__(topic,UInt16MultiArray, lambda msg,*args:callback(pickle.loads(bytes(msg.data)),*args),*args, **kwargs)
        
