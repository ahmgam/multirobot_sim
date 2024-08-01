
#################################
# Encryption Module
################################
import rsa
import os
from cryptography.fernet import Fernet
from base64 import  b64encode, b64decode
import pickle
from math import ceil
from collections import OrderedDict
import base64
class EncryptionModule:
    
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
    def hash(message):
        '''
        hash message using SHA-256
        '''
        if type(message) == str:
          message = message.encode('utf-8')
        if type(message) == dict:
          message = pickle.dumps(message)
        if type(message)== OrderedDict:
          message = pickle.dumps(dict(message))
        return b64encode(rsa.compute_hash(message, 'SHA-256')).decode('ascii')

    @staticmethod
    def sign_hash(message,sk):
      #define private key instance from string
        if type(sk) == str:
            sk = rsa.PrivateKey.load_pkcs1(sk)
        message = b64decode(message.encode('ascii'))
        signature = rsa.sign_hash(message, sk, 'SHA-256')
        return b64encode(signature).decode('ascii')


    @staticmethod
    def sign(message,sk):
        #define private key instance from string
        if type(sk) == str:
            sk = rsa.PrivateKey.load_pkcs1(sk)
        if type(message) == str:
            message = message.encode("utf-8")
        if type(message) == dict :
            message = pickle.dumps(message)
        if type(message)== OrderedDict:
            message = pickle.dumps(dict(message))
        signature = rsa.sign(message, sk, 'SHA-256')
        return b64encode(signature).decode('ascii')
        
    @staticmethod
    def verify(message,signature,pk):
        #define public key instance from string
        if type(pk) == str:
            pk = rsa.PublicKey.load_pkcs1(pk)
        #verify signature
        if type(message) == str:
            message = message.encode("utf-8")
        if type(message) == dict:
            message = pickle.dumps(message)
        if type(message)== OrderedDict:
            message = pickle.dumps(dict(message))
        try : 
          if rsa.verify(message, b64decode(signature.encode('ascii')), pk):
            return True
        except:
          return False
        
    @staticmethod
    def format_public_key(pk):
        #remove new line characters
        #pk = str(pk.save_pkcs1().decode('ascii'))
        pk = base64.b64encode(pk.save_pkcs1()).decode()
        #pk = pk.replace('\n-----END RSA PUBLIC KEY-----\n', '').replace('-----BEGIN RSA PUBLIC KEY-----\n','')
        return pk
    
    @staticmethod
    def format_private_key(sk):
        # Convert the byte string to a normal string
        #sk = str(sk.save_pkcs1().decode('ascii'))
        sk=base64.b64encode(sk.save_pkcs1()).decode()
        # Remove the new line characters and the header and footer
        #sk = sk.replace('\n-----END RSA PRIVATE KEY-----\n', '').replace('-----BEGIN RSA PRIVATE KEY-----\n','')
        return sk
        
    @staticmethod
    def reformat_public_key(pk):
        #return f"-----BEGIN RSA PUBLIC KEY-----\n{str(pk)}\n-----END RSA PUBLIC KEY-----\n"
        return rsa.PublicKey.load_pkcs1(base64.b64decode(pk))
    
    @staticmethod
    def reformat_private_key(sk):
        #return f"-----BEGIN RSA PUBLIC KEY-----\n{str(pk)}\n-----END RSA PUBLIC KEY-----\n"
        return rsa.PrivateKey.load_pkcs1(base64.b64decode(sk))
    
    @staticmethod
    def reconstruct_keys(pk,sk):
        #check if key pairs start with header and end with footer
        #if not str(pk).startswith("-----BEGIN RSA PUBLIC KEY-----\n"):
        #    pk = f"-----BEGIN RSA PUBLIC KEY-----\n{str(pk)}"
        #if not str(pk).endswith("\n-----END RSA PUBLIC KEY-----\n"):
        #    pk = f"{str(pk)}\n-----END RSA PUBLIC KEY-----\n"
        #if not str(sk).startswith("-----BEGIN RSA PRIVATE KEY-----\n"):
        #    sk = f"-----BEGIN RSA PRIVATE KEY-----\n{str(sk)}"
        #if not str(sk).endswith("\n-----END RSA PRIVATE KEY-----\n"):
        #    sk = f"{str(sk)}\n-----END RSA PRIVATE KEY-----\n"
        #now load keys 
        pk = rsa.PublicKey.load_pkcs1(base64.b64decode(pk))
        sk = rsa.PrivateKey.load_pkcs1(base64.b64decode(sk))
        return pk, sk
       
    @staticmethod 
    def generate_symmetric_key():
        return Fernet.generate_key().decode("ascii")
         
    @staticmethod
    def encrypt(message, pk):
        if type(pk) == str:
            pk = rsa.PublicKey.load_pkcs1(base64.b64decode(pk))
        if type(message) == str:
            message = message.encode("utf-8")
        if type(message) == dict:
            message = pickle.dumps(message)
        if type(message)== OrderedDict:
            message = pickle.dumps(dict(message))
        #encrypt message
        result = []
        for i in range (ceil(len(message)/245)):
            start_index = i*245
            end_index = (i+1)*245 if (i+1)*245 < len(message) else len(message)
            result.append(rsa.encrypt(message[start_index:end_index], pk))   
        return b64encode(b''.join(result)).decode('utf-8')
    
    @staticmethod
    def decrypt(message,sk):
        #decrypt message
        message = b64decode(message.encode('utf-8'))
        try:
            result = []
            for i in range (ceil(len(message)/256)):
                start_index = i*256
                end_index = (i+1)*256 if (i+1)*256 < len(message) else len(message)
                buffer = message[start_index:end_index]
                result.append(rsa.decrypt(buffer, sk)) 
            result = b''.join(result)
        except Exception as e:
            print(f"error decrypting message: {e.with_traceback()}")
            return None
        #test the type of resposne
        try:
          response = result.decode("ascii")
        except Exception as e:
          #use pickle
          response = pickle.loads(result)
        return response
    
    @staticmethod
    def encrypt_symmetric(message,key):
        if type(message) == str:
            message = message.encode("utf-8")
        if type(message) == dict:
            message = pickle.dumps(message)
        if type(message)== OrderedDict:
            message = pickle.dumps(dict(message))
        f = Fernet(key.encode("ascii"))
        return b64encode(f.encrypt(message)).decode('utf-8')
    
    @staticmethod
    def decrypt_symmetric(ciphertext,key):
        f = Fernet(key.encode("ascii"))
        decrypted = f.decrypt(b64decode(ciphertext.encode('utf-8')))
        try :
          response = decrypted.decode("ascii")
        except Exception as e:
          #use pickle
          response = pickle.loads(decrypted)
        return response
    