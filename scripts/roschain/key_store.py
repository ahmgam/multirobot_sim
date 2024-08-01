#!/usr/bin/env python

#################################
# Encryption Module
################################
import rsa
import base64
import os
from cryptography.fernet import Fernet
import json
from rospy import Service,init_node,get_namespace,get_param,ROSInterruptException,spin,loginfo
from multirobot_sim.srv import FunctionCall,FunctionCallResponse

class EncryptionManager:
    def __init__(self, node_id,public_key_file, private_key_file):
        self.node_id = node_id
        self.public_key_file = public_key_file
        self.private_key_file = private_key_file
        self.pk, self.sk = self.load_keys(public_key_file, private_key_file)
        if self.pk is None or self.sk is None:
            self.pk, self.sk = self.generate_keys()
            self.store_keys(public_key_file, private_key_file, self.pk, self.sk)
        self.node = init_node("key_store", anonymous=True)
        loginfo(f"{self.node_id}: key_store: Initializing key store service")
        self.server = Service(f"/{self.node_id}/key_store/call", FunctionCall, self.handle_function_call)
        loginfo(f"{self.node_id}: key_store:Initialized successfully")
        
    @staticmethod
    def generate_keys():
        '''
        generate new public and private key pair
        '''
        
        #generate new public and private key pair
        pk, sk=rsa.newkeys(2048)
        return pk, sk
    
    @staticmethod
    def store_keys(public_key_file,private_key_file,pk,sk):
        '''
        store public and private key pair in file
        '''
        
        #store public and private key pair in file
        # Save the public key to a file
        with open(public_key_file, 'wb') as f:
            f.write(pk.save_pkcs1())

        # Save the private key to a file
        with open(private_key_file, 'wb') as f:
            f.write(sk.save_pkcs1())
        return None
    
    @staticmethod
    def load_keys(pk_file,sk_file):
        '''
        load public and private key pair from file
        '''
        #check if key pairs is available
        if os.path.isfile(pk_file) and os.path.isfile(sk_file):
            #load public and private key pair from file
            with open(pk_file, 'rb') as f:
                pk = rsa.PublicKey.load_pkcs1(f.read())
            with open(sk_file, 'rb') as f:
                sk = rsa.PrivateKey.load_pkcs1(f.read())
            return pk, sk
        else:        
            return None, None
        
        
    @staticmethod
    def format_public_key(pk):
        pk = base64.b64encode(pk.save_pkcs1()).decode()
        return pk
    
    @staticmethod
    def format_private_key(sk):
        sk=base64.b64encode(sk.save_pkcs1()).decode()
        return sk
        
    @staticmethod 
    def generate_symmetric_key():
        return Fernet.generate_key().decode("ascii")
         
    def get_rsa_key(self):
        return {
            "pk": self.format_public_key(self.pk),
            "sk": self.format_private_key(self.sk)
        }
        
    def handle_function_call(self, req):
        if req.function_name == "get_rsa_key":
            return FunctionCallResponse(json.dumps(self.get_rsa_key()))
        else:
            return FunctionCallResponse(r"{}")
        
if __name__ == "__main__":
    ns = get_namespace()
    try :
        node_id= get_param(f'{ns}key_store/node_id') # node_name/argsname
        loginfo(f"key_store: Getting node_id argument, and got : {node_id}")
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : node_id")
    try :
        public_key_file= get_param(f'{ns}key_store/public_key_file') # node_name/argsname
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : public_key_file")
    
    try:
        private_key_file= get_param(f'{ns}key_store/private_key_file') # node_name/argsname
    except ROSInterruptException:
        raise ROSInterruptException("Invalid arguments : private_key_file")
    
    network = EncryptionManager(node_id,public_key_file,private_key_file)
    spin()
    