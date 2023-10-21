#!/usr/bin/env python
import rsa
import os
from cryptography.fernet import Fernet
from base64 import  b64encode, b64decode
import json
from math import ceil
from rospy import loginfo
import datetime
from std_srvs.srv import Trigger, TriggerResponse
from queue import Queue
import rospy
import queue
from collections import OrderedDict
from time import mktime
from random import choices
from string import digits, ascii_uppercase,ascii_lowercase
from paho.mqtt import client as mqtt_client
from multirobot_sim.srv import GetBCRecords,SubmitTransaction,GetBCRecordsResponse,SubmitTransactionResponse, DatabaseQuery, DatabaseQueryRequest
from time import sleep


#################################
# Encryption Module
################################

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
        if type(message) == dict:
          message = json.dumps(message)
        return b64encode(rsa.compute_hash(message.encode('utf-8'), 'SHA-256')).decode('ascii')

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
        if type(message) == dict:
            message = json.dumps(message)
        signature = rsa.sign(message.encode("utf-8"), sk, 'SHA-256')
        return b64encode(signature).decode('ascii')
        
    @staticmethod
    def verify(message,signature,pk):
        #define public key instance from string
        if type(pk) == str:
            pk = rsa.PublicKey.load_pkcs1(pk)
        #verify signature
        if type(message) == dict:
            message = json.dumps(message)
        try : 
          if rsa.verify(message.encode("utf-8"), b64decode(signature.encode('ascii')), pk):
            return True
        except:
          return False
        
    @staticmethod
    def format_public_key(pk):
        #remove new line characters
        pk = str(pk.save_pkcs1().decode('ascii'))
        pk = pk.replace('\n-----END RSA PUBLIC KEY-----\n', '').replace('-----BEGIN RSA PUBLIC KEY-----\n','')
        return pk
        
    @staticmethod
    def reformat_public_key(pk):
        return f"-----BEGIN RSA PUBLIC KEY-----\n{str(pk)}\n-----END RSA PUBLIC KEY-----\n"
       
    @staticmethod 
    def generate_symmetric_key():
        return Fernet.generate_key().decode("ascii")
         
    @staticmethod
    def encrypt(message, pk):
        if type(pk) == str:
            pk = rsa.PublicKey.load_pkcs1(pk)
        #encrypt message
        result = []
        for i in range (ceil(len(message)/245)):
            start_index = i*245
            end_index = (i+1)*245 if (i+1)*245 < len(message) else len(message)
            result.append(rsa.encrypt(message[start_index:end_index].encode("ascii"), pk))   
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
                result.append(rsa.decrypt(message[start_index:end_index], sk).decode("ascii"))   
            return ''.join(result)
        except Exception as e:
            loginfo(f"error decrypting message: {e}")
            return None
    
    @staticmethod
    def encrypt_symmetric(message,key):
        f = Fernet(key.encode("ascii"))
        return b64encode(f.encrypt(message.encode("utf-8"))).decode('utf-8')
    
    @staticmethod
    def decrypt_symmetric(ciphertext,key):
        f = Fernet(key.encode("ascii"))
        return f.decrypt(b64decode(ciphertext.encode('utf-8'))).decode("ascii")
    
###################################
# RabbitMQ Communication Module
###################################

class MQTTCommunicationModule:
    def __init__(self,node_id,endpoint,port,auth=None,DEBUG=False):
        self.node_id = node_id
        self.endpoint = endpoint
        self.port = port
        self.auth = auth
        self.DEBUG = DEBUG
        self.base_topic = "nodes"
        self.log_topic = 'logs'
        self.buffer = Queue()
        self.__init_mqtt()
        self.counter = 0
        self.timeout = 5

    def __init_mqtt(self):
        self.client = mqtt_client.Client(self.node_id)
        if self.auth is not None:
            self.client.username_pw_set(self.auth["username"],self.auth["password"])
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(self.endpoint, self.port)
            self.client.subscribe(f"{self.base_topic}/{self.node_id}")
            self.client.subscribe(f"{self.base_topic}")
        except Exception as e:
            rospy.loginfo(f"{node_id}: Error connecting to MQTT: {e}")
            return

    def on_message(self, client, userdata, message):
        self.buffer.put({"message":json.loads(message.payload.decode("utf-8")),"type":"incoming"})

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"{self.node_id}: Connected with result code " + str(rc))
        self.client.subscribe(f"{self.base_topic}/{self.node_id}")
        self.client.subscribe(f"{self.base_topic}")
        
    def send(self, message):
        if self.DEBUG:
            rospy.loginfo(f'{self.node_id}: Sending message to {message["target"]} with type {message["message"]["type"]}')
        #parse message to string
        if type(message["message"]) == OrderedDict or type(message["message"]) == dict:
          message["message"] = json.dumps(message["message"])
        else:
          message["message"] = str(message["message"])
        try:
            if message["target"] == "all":
                self.client.publish(f"{self.base_topic}", message["message"])
            else:
                self.client.publish(f"{self.base_topic}/{message['target']}", message["message"])
            self.counter += 1
            return True
        except Exception as e:
            rospy.loginfo(f"{self.node_id}: Error sending message: {e}")
            return False
        
    def send_log(self,message):
        self.client.publish(f"{self.log_topic}", f"{self.node_id}|{message}")

    def get(self):
        #self.client.loop()
        if self.is_available():
            return self.buffer.get()
        else:
            return None
        
    def is_available(self):
        self.client.loop_read()
        return not self.buffer.empty()

####################################
# Database module
###################################

class Database (object):
    def __init__(self,node_id):
        #self.working = False
        self.node_id = node_id
        self.query_client = rospy.ServiceProxy(f"{self.node_id}/query", DatabaseQuery)
        self.query_client.wait_for_service()
        self.tabels = self.__get_db_meta()
        

    def __get_db_meta(self):
        cols = self.query("""
        SELECT 
        m.name as table_name, 
        p.name as column_name,
        p.type as column_type,
        p.'notnull' as not_null
        FROM 
        sqlite_master AS m
        JOIN 
        pragma_table_info(m.name) AS p
        WHERE
        m.type = 'table' 
        ORDER BY 
        m.name, 
        p.cid
        """)
        tabels = {table_name : {"name":table_name,"columns":{}} for table_name in set([col['table_name'] for col in cols])}
        # add columns to tabels
        for col in cols:
            tabels[col['table_name']]["columns"][col['column_name']]=({"name":col['column_name'],"type":col['column_type'], "not_null":col['not_null']})

        #remove sqlite_sequence table
        tabels.pop("sqlite_sequence",None)
        #replace type with python type
        for table_name,table_content in tabels.items():
            for column in table_content["columns"].values():
                if column["type"] == "INTEGER":
                    tabels[table_name]["columns"][column["name"]]["type"] = int
                elif column["type"] == "REAL":
                    tabels[table_name]["columns"][column["name"]]["type"] =float
                elif column["type"] == "TEXT":
                    tabels[table_name]["columns"][column["name"]]["type"] = str
                elif column["type"] == "BLOB":
                    tabels[table_name]["columns"][column["name"]]["type"] = bytes
                else:
                    raise Exception("Column type not supported")
        return tabels

    def __table_exists(self,table_name):
        return table_name in self.tabels.keys()
    
    def __column_exists(self,table_name,column_name):
        return column_name in self.tabels[table_name]["columns"].keys() or column_name=="*"
    
    def __check_fields_format(self,fields):
        if not type(fields) in [list,tuple]:
            raise Exception("Column must be a list or tuple")
        if len(fields) != 2:
            raise Exception("Column must have 2 elements")

    def __check_condition_format(self,conditions):
        if not type(conditions) in [list,tuple]:
            raise Exception("Column must be a list or tuple")
        if len(conditions) != 3:
            raise Exception("Column must have 3 elements")
         
    def __check_column_options(self,column):
        if not column[1] in ["==",">=","<=",">","<","!=","LIKE","NOT LIKE","IN","NOT IN","IS","IS NOT","BETWEEN","NOT BETWEEN","NULL","NOT NULL"]:
            raise Exception("Column type not supported")
        
    def __check_column_type(self,table,column,value):
        if not type(value) in [int,float,str,bytes,bool,None]:
            raise Exception(f"Column type not supported : {type(value)}")
        if str(value).isnumeric():
            return
        if not type(value) == self.tabels[table]["columns"][column]["type"]:
            raise Exception(f"Wrong data type for {column} ,data type : {type(value)} , expected : {self.tabels[table]['columns'][column]['type']}")
        
    def insert(self,table_name,*keywords):
        #check if table exists
        if not self.__table_exists(table_name):
            raise Exception(f"Table does not exists : {table_name}")
        #check if fields format is valid
        for keyword in keywords:
            self.__check_fields_format(keyword)
        #check if fields are valid
        for keyword,value in keywords:
            if not self.__column_exists(table_name,keyword):
                raise Exception(f"Column does not exists : {keyword}")
            self.__check_column_type(table_name,keyword,value)

        #build query
        query = "INSERT INTO {table} ({keywords}) VALUES ({values})".format(
            table=table_name,keywords=",".join(keyword[0] for keyword in keywords),
            values=",".join(str(keyword[1]) if type(keyword[1]) != str else f"'{keyword[1]}'" for keyword in keywords))
            
        #execute query
        self.query(query)
       
        return 
        
    def flush(self):
        for table_name in self.tabels.keys():
            self.query(f"DROP TABLE IF EXISTS {table_name}")

    def select(self,table_name,fields,*conditions):
        
        #check if table exists
        if not self.__table_exists(table_name):
            raise Exception(f"Table does not exists : {table_name}")
        #check if fields exists
        if type(fields) == str:
            fields = [fields]
        for field in fields:
            if not self.__column_exists(table_name,field):
                raise Exception(f"Column does not exists : {field}")
        #check if conditions are valid
        for condition in conditions:
            self.__check_condition_format(condition)
            if not self.__column_exists(table_name,condition[0]):
                raise Exception(f"Column does not exists : {condition[0]}")
            self.__check_column_options(condition)
            self.__check_column_type(table_name,condition[0],condition[2])
       
        #build query
        query = "SELECT {fields} FROM {table} {options}".format(
            fields=",".join(fields),table=table_name,
            options="WHERE "+" AND ".join([f"{condition[0]} {condition[1]} {condition[2]}" if type(condition[2]) != str else f"{condition[0]} {condition[1]} '{condition[2]}'" for condition in conditions]) if len(conditions) > 0  else ""
            )
     
        return self.query(query)
    
    def delete(self,table_name,*conditions):

        #check if table exists
        if not self.__table_exists(table_name):
            raise Exception(f"Table does not exists : {table_name}")

        #check if conditions are valid
        for condition in conditions:
            self.__check_condition_format(condition)
            if not self.__column_exists(table_name,condition[0]):
                raise Exception(f"Column does not exists : {condition[0]}")
            self.__check_column_options(condition)
            self.__check_column_type(table_name,condition[0],condition[2])

        #build query
        query = "DELETE FROM {table} {options}".format(
            table=table_name,
            options="WHERE "+" AND ".join([f"{condition[0]} {condition[1]} {condition[2]}" for condition in conditions]) if conditions else ""
            )
        #execute query
        return self.query(query)
    
    def update(self,table_name,*conditions,**keyword):

        #check if table exists
        if not self.__table_exists(table_name):
            raise Exception(f"Table does not exists : {table_name}")
        #check if conditions are valid
        for condition in conditions:
            self.__check_condition_format(condition)
            if not self.__column_exists(table_name,condition[0]):
                raise Exception(f"Column does not exists : {condition[0]}")
            self.__check_column_options(condition)
            self.__check_column_type(table_name,condition[0],condition[2])
        #check keywords
        for key,value in keyword.items():
            if not self.__column_exists(table_name,key):
                raise Exception(f"Column does not exists : {key}")
            self.__check_column_type(table_name,key,value)

        #build query
        query = "UPDATE {table} SET {keywords} {options}".format(
            table=table_name,
            keywords=",".join([f"{key} = {value}" for key,value in keyword.items()]),
            options="WHERE "+" AND ".join([f"{condition[0]} {condition[1]} {condition[2]}" for condition in conditions]) if conditions else ""
            )
        #rospy.loginfo(query)
        #execute query
        return self.query(query)
    
    def count(self,table_name,*conditions):
        
        #check if table exists
        if not self.__table_exists(table_name):
            raise Exception(f"Table does not exists : {table_name}")
        #check if conditions are empty
        if not conditions:
            return self.query(f"SELECT COUNT(*) FROM {table_name}")
        #check if conditions are valid
        for condition in conditions:
            self.__check_condition_format(condition)
            if not self.__column_exists(table_name,condition[0]):
                raise Exception(f"Column does not exists : {condition[0]}")
            self.__check_column_options(condition)
            self.__check_column_type(table_name,condition[0],condition[2])
       
        #build query
        query = "SELECT COUNT(*) FROM {table} {options}".format(
            table=table_name,
            options="WHERE "+" AND ".join([f"{condition[0]} {condition[1]} {condition[2]}" for condition in conditions]) if conditions else ""
            )
        #execute query
        return self.query(query)
 
    def get_last_id(self,table_name):
        #check if table exists
        if not self.__table_exists(table_name):
            raise Exception(f"Table does not exists : {table_name}")
        #check if table is empty
        if not self.query(f"SELECT * FROM {table_name}"):
            return 0
        return self.query(f"SELECT MAX(id) FROM '{table_name}'")[0]['MAX(id)']
         
    def query(self, query):   
     
        result = self.query_client(DatabaseQueryRequest(query))
        data = []
        if result.id == 0:
            data = []
            for i in range(len(result.output)):
                #parse json without raising exception
                data.append(json.loads(result.output[i],strict=False))
            return data
        else:
            return result.id
        
    
    def update_db_meta(self):
        self.tabels = self.__get_db_meta()

#########################################
# Blockchain module
#########################################


class Blockchain:
    #initialize the blockchain
    def __init__(self,parent):
        
        #define parent
        rospy.loginfo(f"{parent.node_id}: blockchain: Initializing")
        self.parent = parent
        # define database manager
        self.db = Database(self.parent.node_id)
        rospy.loginfo(f"{parent.node_id}: blockchain: Initializing database")
        # create tables
        self.create_tables()
        # define queue for storing data
        rospy.loginfo(f"{parent.node_id}: blockchain: Initializing queue")
        self.genesis_block()
        #sync timeout
        rospy.loginfo(f"{parent.node_id}: blockchain: Initializing sync timeout")
        self.sync_timeout = 10
        #sync views
        self.views = OrderedDict()
        
 
    ############################################################
    # Database tabels
    ############################################################
    def create_tables(self):
        
        #create block table
        block_table_query = """
        CREATE TABLE IF NOT EXISTS block (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            tx_start_id INTEGER NOT NULL,
            tx_end_id INTEGER NOT NULL,
            merkle_root TEXT NOT NULL,
            combined_hash TEXT NOT NULL,
            timecreated TEXT NOT NULL
        );"""
        self.db.query(block_table_query)
        #create transaction table
        transaction_table_query = """
        CREATE TABLE IF NOT EXISTS transactions (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            item_id INTEGER  NOT NULL,
            item_table TEXT NOT NULL,
            hash TEXT NOT NULL,
            timecreated TEXT NOT NULL
        );"""
        self.db.query(transaction_table_query)
        self.db.update_db_meta()

    ############################################################
    # blockchain operations
    ############################################################
    
    #create the genesis block
    def genesis_block(self):
        #add genesis transaction to the blockchain containing 
        #get previous hash
        prev_hash = self.__get_previous_hash()
        #combine the hashes
        combined_hash = self.__get_combined_hash(prev_hash,prev_hash)
        #add the transaction to the blockchain
        self.db.insert("block",("tx_start_id",0),("tx_end_id",0),("merkle_root",prev_hash),("combined_hash",combined_hash),("timecreated",datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
    
    def add_sync_record(self,block):
        pass
        #check if block exists in the blockchain
        block_exists = self.db.select("block", ["id", "combined_hash"], ("id", "==", block["metadata"]["id"]))
        if block_exists:
            if block_exists[0]["combined_hash"] == block["metadata"]["combined_hash"]:
                return
            else:
                #update block
                self.db.update(
                    "block",
                    ("id", block["metadata"]["id"]),
                    tx_start_id=block["metadata"]["tx_start_id"],
                    tx_end_id=block["metadata"]["tx_end_id"],
                    merkle_root=block["metadata"]["merkle_root"],
                    combined_hash=block["metadata"]["combined_hash"],
                    timecreated=block["metadata"]["timecreated"],
                    )
        else:
            #add block
            self.db.insert(
                "block",
                ("id",block["metadata"]["id"]),
                ("tx_start_id",block["metadata"]["tx_start_id"]),
                ("tx_end_id",block["metadata"]["tx_end_id"]),
                ("merkle_root",block["metadata"]["merkle_root"]),
                ("combined_hash",block["metadata"]["combined_hash"]),
                ("timecreated",block["metadata"]["timecreated"]),
            )
        #insert transactions
        for transaction,record in block["transactions"].values():
            #check if transaction exists
            transaction_exists = self.db.select("transactions", ["id","hash"], ("id", "==", record["id"]))
            if transaction_exists:
                if transaction_exists[0]["hash"] == transaction["hash"]:
                    continue
                else:
                    #update transaction
                    self.db.update(
                        "transactions",
                        ("id",transaction["id"]),
                        item_id=transaction["item_id"],
                        item_table=transaction["item_table"],
                        hash=transaction["hash"],
                        timecreated=transaction["timecreated"],
                        )
                    #update the record
                    self.db.update(
                        transaction["item_table"],
                        ("id",record["id"]),
                        **record
                    )
            else:
                #insert transaction
                self.db.insert(
                    "transactions",
                    ("id",transaction["id"]),
                    ("item_id",transaction["item_id"]),
                    ("item_table",transaction["item_table"]),
                    ("hash",transaction["hash"]),
                    ("timecreated",transaction["timecreated"])
                    )
                #insert the record 
                self.db.insert(
                    transaction["item_table"],
                    *[(key,value) for key,value in record.items()]
                )

    #commit a new transaction to the blockchain
    def add_transaction(self,table,data,time =mktime(datetime.datetime.now().timetuple())):
        
        #add the record to it's table
        self.db.insert(table,*[(key,value) for key,value in data.items()])
        #get the inserted record
        item = self.db.select(table,["*"],*[(key,'==',value) for key,value in data.items()])[0]
        item_id = item["id"]
        #remove the hash from the record
        last_transaction_id = self.db.get_last_id("transactions")
        current_hash = self.__get_current_hash(item)
        #add the transaction to the blockchain
        time_created = datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S") 
        self.db.insert("transactions",("item_id",item_id),("item_table",table),("hash",current_hash),("timecreated",time_created))
        #sending log info 
        self.parent.comm.send_log(f"{table}({last_transaction_id+1})")
        return item_id

    def add_block(self,transactions):
        ids = []
        for transaction in transactions:
        
            id =self.add_transaction(transaction["message"]["table_name"],json.loads(transaction["message"]["data"]),transaction["message"]["time"])
            ids.append(id)
        #sord ids list
        ids.sort()
        #get all meta data of transactions
        transactions_meta= []
        for id in ids:
            transaction_meta = self.get_metadata(id)
            transactions_meta.append(transaction_meta)
        root = self.__get_merkle_root(transactions_meta)
        #get last id of block
        last_block_id = self.db.get_last_id("block")
        #get previous hash
        prev_hash = self.__get_previous_hash(last_block_id)
        #combine the hashes
        combined_hash = self.__get_combined_hash(root,prev_hash)
        #add the transaction to the blockchain
        self.db.insert("block",("tx_start_id",min(ids)),("tx_end_id",max(ids)),("merkle_root",root),("combined_hash",combined_hash),("timecreated",datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
    
    def get_transaction(self,transaction_id):
        transaction_data = self.get_metadata(transaction_id)
        if not transaction_data[0]:
            return None,None
        item_data = self.get_record(transaction_data["item_table"],transaction_data["item_id"])
        if not item_data:
            return None,None
        return transaction_data,item_data
    
    def get_metadata(self,transaction_id):
        transaction_data = self.db.select("transactions",["*"],("id",'==',transaction_id))
        if not transaction_data:
            return None,None
        else:
            return transaction_data[0]
    
    def get_record(self,table,record_id):
        return self.db.select(table,["*"],{"id":record_id})[0]
    
    def filter_records(self,table,filter):
        return self.db.select(table,["*"],filter)
    
    def get_blockchain(self,start_id=None,end_id = None):
        if start_id is None or start_id < 0:
            start_id = 0
        if end_id is None or end_id > self.db.get_last_id("blockchain"):
            end_id = self.db.get_last_id("blockchain")
        blockchain = []
        for i in range(start_id,end_id+1):
            blockchain.append(self.get_transaction(i))
        return blockchain

    def __get_previous_hash(self,last_transaction_id=None):
        
        if last_transaction_id is None:
            #add genesis transaction, get the hash of auth data
            prev_hash = EncryptionModule.hash(json.dumps(self.parent.auth))
        else:
            #get the hash of last transaction
            prev_hash = self.db.select("block",["combined_hash"],("id",'==',last_transaction_id))[0]["combined_hash"]
        return prev_hash
    
    def __get_current_hash(self,item):
        #remove the hash from the record
        current_hash = EncryptionModule.hash(json.dumps(item, sort_keys=True))
        return current_hash
    
    def __get_combined_hash(self,current_hash,prev_hash):
        #combine the hashes
        combined_hash = EncryptionModule.hash(current_hash+prev_hash)
        return combined_hash
    #check if the blockchain is valid
    
    def __get_merkle_root(self,data):

            
        if len(data) == 0:
            return None

        # Initialize a list to hold the current level of hashes
        current_level = [self.__get_current_hash(d) for d in data]

        while len(current_level) > 1:
            next_level = []

            # Iterate through pairs of hashes, hash them together, and add to the next level
            i = 0
            while i < len(current_level):
                if i + 1 < len(current_level):
                    combined_hash = self.__get_current_hash(current_level[i] + current_level[i + 1])
                    next_level.append(combined_hash)
                else:
                    # If there's an odd number of hashes, hash the last one with itself
                    combined_hash = self.__get_current_hash(current_level[i] + current_level[i])
                    next_level.append(combined_hash)
                i += 2

            current_level = next_level

        return current_level[0]



    def validate_chain(self,start_id = None,end_id = None):
        if start_id is None or start_id < 0:
            start_id = 0
        if end_id is None or end_id > self.db.get_last_id("block"):
            end_id = self.db.get_last_id("block")
        for i in range(start_id,end_id+1):
            if not self.validate_block(i):
                return False
        return True

    def validate_block(self,block_id):
        #define validation result
        block_valid = False
        transactions_valid = True
        merkle_root_valid = False
        #get block data 
        block = self.db.select("block",["*"],("id",'==',block_id))[0]
        #get previous block 
        previous_hash = self.__get_previous_hash(block_id-1)
        #compare the hashes
        if block["combined_hash"] == self.__get_combined_hash(block["merkle_root"],previous_hash):
            block_valid= True
        #get all meta data of transactions
        transactions_meta= []
        for id in range(block["start_id"],block["end_id"]+1):
            #get the transaction
            transaction_data,item_data = self.get_transaction(id)
            #get the current hash
            current_hash = self.__get_current_hash(item_data)
            #check if the combined hash is equal to the combined hash in the blockchain
            if current_hash != transaction_data["hash"]:
                transactions_valid = False
            transactions_meta.append(transaction_data)
        #compare merkle roots
        if self.__get_merkle_root(transactions_meta) == block["merkle_root"]:
            transactions_valid = True
        #check if the block is valid
        if block_valid and transactions_valid:
            merkle_root_valid = True
        return block_valid,transactions_valid,merkle_root_valid
        
    ############################################################
    # Syncing the blockchain with other nodes
    ############################################################
    #send sync request to other nodes
    def cron(self):
        #TODO implement cron for view timeout
        #check views for timeout
        for view_id,view in self.views.copy().items():
            if mktime(datetime.datetime.now().timetuple()) - view['last_updated'] > self.sync_timeout and view['status'] == "pending":
                #evaluate the view
                self.evaluate_sync_view(view_id)
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: View {view_id} timed out, starting evaluation")
              
    def check_sync(self,last_conbined_hash, record_count):
        #check if all input is not null 
        if  last_conbined_hash is None or record_count == 0:
            return True 
   
        #check if last combined hash exists in the blockchain
        last_record = self.db.select("block",["id","combined_hash"],("combined_hash",'==',last_conbined_hash))
        if len(last_record) == 0:
            #if not, then return false
            #end_id = self.db.get_last_id("blockchain")
            #end_hash = self.db.select("blockchain",["combined_hash"],("id",'==',end_id))[0]["combined_hash"]
            return False        
        else:
            #get the id of the last record
            end_id = last_record[0]["id"]

        #check if the number of records is equal to the number of records in the blockchain
        if record_count != self.db.get_last_id("block"):
            return False
        return True
    
        
    def get_sync_info(self):
        last_id = self.db.get_last_id("block")
        last_record = self.db.select("block",["combined_hash"],("id",'==',last_id))
        if len(last_record) == 0:
            last_record = None
        else:
            last_record = last_record[0]["combined_hash"]
        number_of_records = self.db.get_last_id("block")
        return last_record,number_of_records
    
    def get_sync_data(self,end_hash,record_count):
        #get end id
 
        end_id = self.db.select("block",["id"],("combined_hash",'==',end_hash))
        if len(end_id) == 0 or end_hash is None:
            end_id = self.db.get_last_id("block")
            start_id = 1
        else:
            end_id = end_id[0]["id"]
            if end_id == self.db.get_last_id("block"):
                return []
            elif end_id != record_count:
                start_id = 1
                end_id = self.db.get_last_id("block")
            else:
                start_id = end_id + 1
                end_id = self.db.get_last_id("block")
        #get the blockchain between start and end id
        blockchain = {}
        blocks = self.db.select("block",["*"],("id",">=",start_id),("id","<=",end_id))
        for block in blocks:
            #get the item
            blockchain[block["id"]]={}
            blockchain[block["id"]]["metadata"] = block
            blockchain[block["id"]]["transactions"]= {}
            #get the transactions
            transactions = self.db.select("blockchain",["*"],("id",">=",block["tx_start_id"]),("id","<=",block["tx_end_id"]))
            for transaction in transactions:
                record = self.get_record(block["item_table"],block["item_id"])
                blockchain[block["id"]]["transactions"][transaction["id"]]=(record,transaction)

        return blockchain
    
    def send_sync_request(self):
        #get the sync info
        last_record,number_of_records = self.get_sync_info()
        #add sync view
        view_id = EncryptionModule.hash(str(last_record)+str(number_of_records)+str(mktime(datetime.datetime.now().timetuple())))
        self.views[view_id] = {
            "last_updated":mktime(datetime.datetime.now().timetuple()),
            "last_record":last_record,
            "number_of_records":number_of_records,
            "status":"pending",
            "sync_data":[]
        }

        msg = {
            "operation":"sync_request",
            "last_record":last_record,
            "number_of_records":number_of_records,
            "view_id":view_id,
            "source":self.parent.node_id
        }
        #send the sync request to other nodes
        self.parent.network.send_message('all',msg)

    #handle sync request from other nodes
    def handle_sync_request(self,msg):
        #get last hash and number of records
        node_id = msg["source"]
        last_record = msg["last_record"]
        number_of_records = msg["number_of_records"]
        view_id = msg["view_id"]
        #check if the blockchain is in sync
        if self.check_sync(last_record,number_of_records):
            #if it is, then send a sync reply
            msg = {
                "operation":"sync_reply",
                "last_record":last_record,
                "number_of_records":number_of_records,
                "sync_data":self.get_sync_data(last_record,number_of_records),
                "view_id":view_id,
                "source":self.parent.node_id
            }
            self.parent.network.send_message(node_id,msg)
 
    def handle_sync_reply(self,msg):
        #check if the view exists
        view_id = msg["message"]["data"]["view_id"]
        if view_id in self.views.keys():
            #if it does, then add the sync data to the view
            self.views[view_id]["sync_data"].append(msg["message"]["data"]["sync_data"])
            #check if the number of sync data is equal to the number of nodes
            if len(self.views[view_id]["sync_data"]) == len(self.parent.sessions.get_connection_sessions()):
                self.evaluate_sync_view(view_id)
        else:
            rospy.loginfo(f"{self.parent.node_id}: view does not exist")

    def evaluate_sync_view(self,view_id):
        #check if the view exists
        if view_id not in self.views.keys():
            rospy.loginfo(f"{self.parent.node_id}: view does not exist")
            return
        #check if the view is complete
        if self.views[view_id]["status"] != "pending":
            return
        #check if the number of sync data is more than half of the nodes
        active_nodes = len(self.parent.sessions.get_active_nodes())
        participating_nodes = len(self.views[view_id]['sync_data'])
        print(f"number of sync data : {participating_nodes}")
        print(f"number of nodes : {active_nodes}")
        if len(self.views[view_id]["sync_data"]) < active_nodes//2:
            rospy.loginfo(f"{self.parent.node_id}: not enough sync data")
            #mark the view as incomplete
            self.views[view_id]["status"] = "incomplete"
            return

        #loop through the sync data and add them to dictionary
        sync_records = {}
        for data in self.views[view_id]["sync_data"]:
            for id,item in data.items():
                if f"{id}:{item['combined_hash']}" not in sync_records.keys():
                    sync_records[f"{id}:{item['combined_hash']}"] = {"score":0,"item":item}
                sync_records[f"{id}:{item['combined_hash']}"]["score"] += 1
        
        #loop through the sync records and and delete the ones with the lowest score
        keys = list(sync_records.keys())
        for key in keys:
            if sync_records[key]["score"] < participating_nodes//2:
                del sync_records[key]

        #loop through the sync records and check if each key has the same value for all nodes
        sync_data = [block["item"] for block in sync_records.values()]
        for block in sync_data:
            self.add_sync_record(block["item"])
        #change the status of the view
        self.views[view_id]["status"] = "complete"

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
            rospy.loginfo("required_fields must be a list")
            raise TypeError("required_fields must be a list")    
        for value in required_fields:
            if value not in data["message"]["data"].keys():
                rospy.loginfo("field {} is required".format(value))
                raise ValueError("field {} is required".format(value))

    def __validate(self,data):
        #validate the message
        #if not (isinstance(data,dict) or isinstance(data,OrderedDict)):
        #    raise TypeError("data must be a dictionary")
        for key in ["type","session_id","node_id","message","node_type","pos","port"]:
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
                
###############################################
# Queues managers
###############################################

class QueueManager:
    
    def __init__(self):
        self.queue = []
        self.output_queue = []
        self.consensus_queue = []
        
    def put_queue(self, message,msg_type,on_failure=None):
        
        #add message to queue
        self.queue.append({
            "message": message,
            "time": mktime(datetime.datetime.now().timetuple()) if message.get("time",None) is None else message["time"],
            "type": msg_type,
            "on_failure": on_failure
        })
     
        try:
            self.queue.sort(key=lambda x: x['time'])
        except KeyError as e:
            print(self.queue)
            exit()
                 
    def pop_queue(self):
        #get message from send queue
        if len(self.queue) == 0:
            return None
        else:
            data = self.queue.pop(0)
            return data
        
    @property
    def queue_empty(self):
        return len(self.queue) == 0
    
    @property
    def queue_count(self):
        return len(self.queue)
    
    def put_output_queue(self, message,msg_source,msg_type,timestamp=mktime(datetime.datetime.now().timetuple())):
        
        #add message to queue
        self.output_queue.append({
            "message": message,
            "source": msg_source,
            "type": msg_type,
            "time": timestamp
        })
        self.output_queue.sort(key=lambda x: x['time'])
    
                 
    def pop_output_queue(self):
        #get message from send queue
        if len(self.output_queue)==0:
            return None
        else:
            data = self.output_queue.pop(0)
            return data    
        
    @property
    def output_queue_empty(self):
        return len(self.output_queue) == 0
    
    @property
    def output_queue_count(self):
        return len(self.output_queue)
    
    def put_consensus_queue(self, message,msg_source,msg_type,timestamp=mktime(datetime.datetime.now().timetuple())):
        
        #add message to queue
        self.consensus_queue.append({
            "message": message,
            "source": msg_source,
            "type": msg_type,
            "time": timestamp
        })
        self.consensus_queue.sort(key=lambda x: x['time'])
    
                 
    def pop_consensus_queue(self):
        #get message from send queue
        if len(self.consensus_queue)==0:
            return None
        else:
            data = self.consensus_queue.pop(0)
            return data    
        
    @property
    def consensus_queue_empty(self):
        return len(self.consensus_queue) == 0
    
    @property
    def consensus_queue_count(self):
        return len(self.consensus_queue)
########################################
# Sessions manager
########################################

class SessionManager:
    def __init__(self,parent):
        #define session manager
        self.parent = parent
        self.discovery_sessions =OrderedDict()
        self.connection_sessions = OrderedDict()
        self.node_states = OrderedDict({self.parent.node_id:{"pk":EncryptionModule.format_public_key(self.parent.pk),"last_active":mktime(datetime.datetime.now().timetuple())}})
   
    def create_discovery_session(self, node_id, data):
        
        #create new session with the given public key and type
        data["node_id"] = node_id
        #add last call timestamp
        data["last_active"] = mktime(datetime.datetime.now().timetuple())
        self.discovery_sessions[node_id]= data
            
    def update_discovery_session(self, node_id, data):
        #update session with the given public key and type
        for key,value in data.items():
            self.discovery_sessions[node_id][key] = value
        #update last call timestamp
        self.discovery_sessions[node_id]["last_active"] = mktime(datetime.datetime.now().timetuple())
        
    def get_discovery_session(self, node_id):
        #get all discovery sessions
        session = self.discovery_sessions.get(node_id,None)
        if session:
            #update last call timestamp
            self.discovery_sessions[node_id]["last_active"] = mktime(datetime.datetime.now().timetuple()) 
        return session
    
    def has_active_connection_session(self, node_id):
        #check if session with the given public key is active
        for key,value in self.connection_sessions.items():
            if value["node_id"] == node_id:
                return True
        return False
    
    def get_connection_sessions(self,session_id):
        #get connection sessions
        session= self.connection_sessions.get(session_id,None)
        if session:
            #update last call timestamp
            self.connection_sessions[session_id]["last_active"] = mktime(datetime.datetime.now().timetuple())
        return session
           
    def generate_session_id(self):
        #generate session id, random string of 32 characters
        return ''.join(choices(ascii_uppercase + digits, k=32))
        
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
        self.connection_sessions[session_id]["last_active"] = mktime(datetime.datetime.now().timetuple())
    
    def get_connection_session_by_node_id(self, node_id):
        #get connection session by node id
        for key,value in self.connection_sessions.items():
            if value["node_id"] == node_id:
                return value
        return None
        
    def get_active_nodes(self):
        return [session["node_id"] for session in self.connection_sessions.values() if session["last_active"] > mktime(datetime.datetime.now().timetuple())-60]

    def get_active_nodes_with_pk(self):
        return [{session["node_id"]:session["pk"]} for session in self.connection_sessions.values() if session["last_active"] > mktime(datetime.datetime.now().timetuple())-60]
    
    def get_node_state_table(self):
        #refresh node state table
        self.refresh_node_state_table()
        #get nodes in state table
        response = {}
        for key,value in self.node_states.items():
            if value["last_active"] > mktime(datetime.datetime.now().timetuple())-60:
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
                self.node_states[key]["last_active"] = mktime(datetime.datetime.now().timetuple())
                continue
            #update last call timestamp
            self.node_states[key] = {"pk":value,"last_active":mktime(datetime.datetime.now().timetuple())}
            
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
                self.node_states[value["node_id"]] = {"pk":value["pk"],"last_active":mktime(datetime.datetime.now().timetuple())}

########################################
# Discovery protocol
########################################

class DiscoveryProtocol:
    def __init__(self,parent):
        #define parent
        self.parent = parent
        #define discovery interval
        self.discovery_interval = 10
        #define discovery last call
        self.last_call = mktime(datetime.datetime.now().timetuple())
        
    def cron(self):
        #check if disvoery last call is more than discovery interval
        #rospy.loginfo(f"session time : {mktime(datetime.datetime.now().timetuple()) - self.last_call}")
        if mktime(datetime.datetime.now().timetuple()) - self.last_call > self.discovery_interval:
            #update last call
            self.last_call = mktime(datetime.datetime.now().timetuple())
            #start discovery
            self.discover()
            

    def handle(self,message):
        if message.message["type"] == "discovery_request":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting response_to_discovery")
            self.respond_to_discovery(message)
        elif message.message["type"] == "discovery_response":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting verify_discovery")
            self.verify_discovery(message)
        elif message.message["type"] == "discovery_verification":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting verify_discovery_response")
            self.verify_discovery_response(message)
        elif message.message["type"] == "discovery_verification_response":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting approve_discovery")
            self.approve_discovery(message)
        elif message.message["type"] == "discovery_approval":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting approve_discovery_response")
            self.approve_discovery_response(message)
        elif message.message["type"] == "discovery_approval_response":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting finalize_discovery")
            self.finalize_discovery(message)
        else:
            rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, but no handler found")
    ################################
    # Challenge management
    ################################   
    def generate_challenge(self, length=20):
        return ''.join(choices(ascii_lowercase, k=length))
    
    def solve_challenge(self,challenge):
        solution = EncryptionModule.hash(challenge)
        client_sol = solution[0:len(solution)//2]
        server_sol = solution[len(solution)//2:]
        return client_sol, server_sol
      
    ################################
    # discovery protocol
    ################################
    def discover(self):
        #discover new nodes on the networن       
        #define message payload
        
        payload = OrderedDict({
            "node_id": self.parent.node_id,
            "node_type": self.parent.node_type,
            "pos": self.parent.pos,
            "type": "discovery_request",
            "port": self.parent.port,
            "time":mktime(datetime.datetime.now().timetuple()),
            "session_id": "",
            "message":{
            "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":{
                    "pk": EncryptionModule.format_public_key(self.parent.pk),
                    }
                },
            })
        #stringify the data payload
        msg_data = json.dumps(payload)
        #generate hash and signature
        msg_signature = EncryptionModule.sign(msg_data,self.parent.sk)
        #add hash and signature to the payload
        payload["signature"] = msg_signature
        #create message object
        message = DiscoveryMessage(payload)
        self.parent.queues.put_queue({"target": "all",
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": message.message,
                                      "pos": self.parent.pos}, "outgoing")
        
    def respond_to_discovery(self,message):
        #respond to discovery requests and send challenge
        #first verify the message
        try:
            message = DiscoveryMessage(message.message) 
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: validation error {e}")
            return None
        #verify the message hash 
        buff = message.message
        msg_signature = buff.pop('signature')
        msg_pk =buff["message"]["data"]["pk"]
        #stringify the data payload
        msg_data = json.dumps(buff)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(msg_pk)) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #check if the node is already connected to the network
        if self.parent.sessions.has_active_connection_session(message.message["node_id"]):
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: connection session is already active") 
            return None
        #check if the node has active discovery session with the sender
        if self.parent.sessions.get_discovery_session(message.message["node_id"]):
            if self.parent.DEBUG:    
                rospy.loginfo(f"{self.parent.node_id}: discovery session is already active")
            return None
        else:
            #create new session
            session_data = {
                "pk": msg_pk,
                "role":"server",
                "counter": message.message["message"]["counter"],
                "node_type": message.message["node_type"],     
            }
            self.parent.sessions.create_discovery_session(message.message["node_id"],session_data)
        #prepare discovery response message
        msg_data =OrderedDict( {
                "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":{
                    "pk": EncryptionModule.format_public_key(self.parent.pk)
                    }
                })
        #stringify the message
        msg_data = json.dumps(msg_data)
        #encrypt the message
        data_encrypted = EncryptionModule.encrypt(msg_data,EncryptionModule.reformat_public_key(msg_pk))   
        payload = {
            "node_id": self.parent.node_id,
            "node_type": self.parent.node_type,
            "pos": self.parent.pos,
            "type": "discovery_response",
            "time":mktime(datetime.datetime.now().timetuple()),
            "port": self.parent.port,
            "session_id": "",
            "message": data_encrypted
            }
        #stringify the message
        payload_data = json.dumps(payload)
        #get message hash,signature
        data_signature = EncryptionModule.sign(payload_data,self.parent.sk)
        #add hash and signature to the message
        payload["signature"] = data_signature
        #send the message
        self.parent.queues.put_queue({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload,
                                      "pos": self.parent.pos}, "outgoing")
    
    def verify_discovery(self,message):
        #verify discovery request and send challenge response
        #check if the node is already connected to the network
        if self.parent.sessions.has_active_connection_session(message.message["node_id"]):
            if self.parent.DEBUG:    
                rospy.loginfo(f"{self.parent.node_id}: connection session is already active")
            return None
        #verify the message hash 
        buff = message.message
        msg_signature = buff.pop('signature')
        msg_data=json.dumps(buff)
        #decrypt the message
        try:
            decrypted_data = EncryptionModule.decrypt(message.message["message"],self.parent.sk)
            #parse the message
            decrypted_data = json.loads(decrypted_data)
        except Exception as e:
            if self.parent.DEBUG:    
                rospy.loginfo(f"{self.parent.node_id}: error decrypting and parsing data : {e}")
            return None
        #validate the message
        message.message["message"] = decrypted_data
        try :
            message=DiscoveryResponseMessage(message.message)
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error validating message : {e}")
            return None
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(decrypted_data["data"]["pk"])) == False:
            if self.parent.DEBUG:    
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        try:
            #generate challenge random string
            challenge = self.generate_challenge()
            #solve the challenge
            client_sol, server_sol = self.solve_challenge(challenge)
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error generating challenge : {e}")
            return None
        #create discovery session
        session_data = {
            "pk": decrypted_data["data"]["pk"],
            "role": "client",
            "counter": message.message["message"]["counter"],
            "node_type": message.message["node_type"],
            "challenge": challenge,
            "client_challenge_response": client_sol,
            "server_challenge_response": server_sol
        }
        #create discovery session
        self.parent.sessions.create_discovery_session(message.message["node_id"],session_data)
        #prepare verification message 
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":{
                    "challenge": challenge,
                    "client_challenge_response": client_sol
                    }
                })
        #stringify the message
        msg_data = json.dumps(msg_data)
        #encrypt the message
        data_encrypted = EncryptionModule.encrypt(msg_data,EncryptionModule.reformat_public_key(decrypted_data["data"]["pk"]))
        payload = OrderedDict({
            "node_id": self.parent.node_id,
            "node_type": self.parent.node_type,
            "pos": self.parent.pos,
            "type": "discovery_verification",
            "time":mktime(datetime.datetime.now().timetuple()),
            "port": self.parent.port,
            "session_id": "",
            "message": data_encrypted
            })
        #stringify the payload
        payload_data = json.dumps(payload)
        #get message hash and signature
        data_signature = EncryptionModule.sign(payload_data,self.parent.sk)
        #add hash and signature to the message
        payload["signature"] = data_signature
        #send the message
        self.parent.queues.put_queue({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload,
                                      "pos": self.parent.pos},"outgoing")
 
    def verify_discovery_response(self,message):
        #verify discovery response and add node to the network
        #check if the node is already connected to the network
        if self.parent.sessions.has_active_connection_session(message.message["node_id"]):
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: connection session is already active")
            return None
        #check if the node does not have active discovery session with the sender
        session = self.parent.sessions.get_discovery_session(message.message["node_id"])
        if not session:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: node does not have active discovery session with the sender")
            return None
        
        #verify the message hash 
        buff = message.message
        msg_signature = buff.pop('signature')
        #verify the message hash
        msg_data = json.dumps(buff)
        #get the public key of the sender from the session
        pk = session["pk"]
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(pk)) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #decrypt the message
        try:
            decrypted_data = EncryptionModule.decrypt(message.message["message"],self.parent.sk)
            
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error decrypting and parsing data : {e}")
            return None
        
        #parse the message
        decrypted_data = json.loads(decrypted_data)
        #check if the message counter is valid
        if decrypted_data["counter"] <= session["counter"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: counter not valid")
            return None
        
        #validate the message
        message.message["message"] = decrypted_data
        try :
            message=VerificationMessage(message.message)
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error validating message : {e}")
            return None
        
        #get the challenge from the incoming message
        challenge = decrypted_data["data"]["challenge"]
        #solve the challenge
        client_sol, server_sol = self.solve_challenge(challenge)
        #compare the client challenge response
        if decrypted_data["data"]["client_challenge_response"] != client_sol:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: client challenge response not verified")
            return None
        #update discovery session
        session_data = {
            "pk": pk,
            "role": "server",
            "counter": message.message["message"]["counter"],
            "node_type": message.message["node_type"],
            "challenge": challenge,
            "client_challenge_response": client_sol,
            "server_challenge_response": server_sol
        }
        #update discovery session
        self.parent.sessions.update_discovery_session(message.message["node_id"],session_data)
        #prepare verification message
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":{
                    "challenge": challenge,
                    "server_challenge_response": server_sol
                    }
                })
        
        #stringify the message
        msg_data = json.dumps(msg_data)
        #encrypt the message    
        data_encrypted = EncryptionModule.encrypt(msg_data,EncryptionModule.reformat_public_key(pk))
        payload = OrderedDict({
            "node_id": self.parent.node_id,
            "node_type": self.parent.node_type,
            "pos": self.parent.pos,
            "type": "discovery_verification_response",
            "time":mktime(datetime.datetime.now().timetuple()),
            "port": self.parent.port,
            "session_id": "",
            "message": data_encrypted
            })
        #stringify the payload
        payload_data = json.dumps(payload)
        #get message hash and signature
        data_signature  = EncryptionModule.sign(payload_data,self.parent.sk)
        #add hash and signature to the message
        payload["signature"] = data_signature
        #send the message
        self.parent.queues.put_queue({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload,
                                      "pos": self.parent.pos},"outgoing")

    def approve_discovery(self,message):
        #approve discovery request and send approval response
        #check if the node is already connected to the network
        if self.parent.sessions.has_active_connection_session(message.message["node_id"]):
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: connection session is already active")
            return None
        #check if the node does not have active discovery session with the sender
        session = self.parent.sessions.get_discovery_session(message.message["node_id"])
        if not session:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: node does not have active discovery session with the sender")
            return None
        #get the public key of the sender from the session
        pk = session["pk"]
        #verify the message hash 
        buff = message.message
        msg_signature = buff.pop('signature')
        #verify the message hash
        msg_data = json.dumps(buff)
        #decrypt the message
        try:
            decrypted_data = EncryptionModule.decrypt(message.message["message"],self.parent.sk)
            
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error decrypting and parsing data : {e}")
            return None
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(pk)) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #parse the message
        decrypted_data = json.loads(decrypted_data)
        #check if the message counter is valid
        if decrypted_data["counter"] <= session["counter"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: counter not valid")
            return None
        
        #validate the message
        message.message["message"] = decrypted_data
        try :
            message=VerificationResponseMessage(message.message)
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error validating message : {e}")
            return None
        #compare the client challenge response
        if decrypted_data["data"]["server_challenge_response"] != session["server_challenge_response"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: client challenge response not verified")
            return None
        
        #creating new session with symmetric key and session id
        #first generate symmetric key
        key = EncryptionModule.generate_symmetric_key()
        #get the session id
        session_id = self.parent.sessions.generate_session_id()
        #create new session
        session_data = {
            "pk": pk,
            "node_id": message.message["node_id"],
            "node_type": message.message["node_type"],
            "last_active": mktime(datetime.datetime.now().timetuple()),
            "port": message.message["port"],
            "role": "server",   
            "counter": message.message["message"]["counter"],
            "session_id": session_id,
            "key": key,
            "status": "pending",
            "last_heartbeat": mktime(datetime.datetime.now().timetuple()),
            "approved": False
        }
        self.parent.sessions.create_connection_session(session_id,session_data)
        #prepare approval message
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":{
                    "session_id": session_id,
                    "session_key": key,
                    "test_message": EncryptionModule.encrypt_symmetric("client_test",key)
                    }
                })
        #stringify the message
        msg_data = json.dumps(msg_data)
        #encrypt the message    
        data_encrypted = EncryptionModule.encrypt(msg_data,EncryptionModule.reformat_public_key(pk))
        payload = OrderedDict({
            "node_id": self.parent.node_id,
            "node_type": self.parent.node_type,
            "pos": self.parent.pos,
            "type": "discovery_approval",
            "time":mktime(datetime.datetime.now().timetuple()),
            "port": self.parent.port,
            "session_id": "",
            "message": data_encrypted
            })
        #stringify the payload
        payload_data = json.dumps(payload)
        #get message hash 
        data_signature = EncryptionModule.sign(payload_data, self.parent.sk)
        #add hash and signature to the message
        payload["signature"] = data_signature
        #send the message
        self.parent.queues.put_queue({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload,
                                      "pos": self.parent.pos},"outgoing")
            
    def approve_discovery_response(self,message):
        #approve discovery response and add node to the network
        #check if the node is already connected to the network
        if self.parent.sessions.has_active_connection_session(message.message["node_id"]):
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: connection session is already active")
            return None
        #check if the node does not have active discovery session with the sender
        session = self.parent.sessions.get_discovery_session(message.message["node_id"])
        if not session:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: node does not have active discovery session with the sender")
            return None
        #get the public key of the sender from the session
        pk = session["pk"]
        #verify the message hash 
        buff = message.message
        msg_signature = buff.pop('signature')
        #verify the message hash
        msg_data = json.dumps(buff)
        #decrypt the message
        try:
            decrypted_data = EncryptionModule.decrypt(message.message["message"],self.parent.sk)
            
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error decrypting and parsing data : {e}")
            return None
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(pk)) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #parse the message
        decrypted_data = json.loads(decrypted_data)
        #check if the message counter is valid
        if decrypted_data["counter"] <= session["counter"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: counter not valid")
            return None
        
        #validate the message
        message.message["message"] = decrypted_data
        try :
            message=ApprovalMessage(message.message)
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error validating message : {e}")
            return None
        
        #first generate symmetric key
        key = decrypted_data["data"]["session_key"]
        #get the session id
        session_id = decrypted_data["data"]["session_id"]
        #decrypt the test message
        try:
            decrypted_test = EncryptionModule.decrypt_symmetric(decrypted_data["data"]["test_message"],key)
            if decrypted_test != "client_test":
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: test message not decrypted")
                return None
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error decrypting test message : {e}")
            return None
        #create new session
        session_data = {
            "pk": pk,
            "node_id": message.message["node_id"],
            "node_type": message.message["node_type"],
            "last_active": mktime(datetime.datetime.now().timetuple()),
            "port": message.message["port"],
            "role": "server",   
            "counter": message.message["message"]["counter"],
            "session_id": session_id,
            "key": key,
            "status": "active",
            "last_heartbeat": mktime(datetime.datetime.now().timetuple()),
            "approved": True
        }
        self.parent.sessions.create_connection_session(session_id,session_data)
        #prepare approval message
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":{
                    "session_id": session_id,
                    "test_message": EncryptionModule.encrypt_symmetric("server_test",key)
                    }
                })
        #stringify the message
        msg_data = json.dumps(msg_data)
        #encrypt the message    
        data_encrypted = EncryptionModule.encrypt(msg_data,EncryptionModule.reformat_public_key(pk))
        payload = OrderedDict({
            "node_id": self.parent.node_id,
            "node_type": self.parent.node_type,
            "pos": self.parent.pos,
            "type": "discovery_approval",
            "time":mktime(datetime.datetime.now().timetuple()),
            "port": self.parent.port,
            "session_id": "",
            "message": data_encrypted
            })
        #stringify the payload
        payload_data = json.dumps(payload)
        #get message hash and signature
        data_signature = EncryptionModule.sign(payload_data, self.parent.sk)
        #add hash and signature to the message
        payload["signature"] = data_signature
        #send the message
        self.parent.queues.put_queue({"target": message.message["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload,
                                      "pos": self.parent.pos},"outgoing")

    def finalize_discovery(self,message):
        #approve discovery response and add node to the network
        #check if the node does not have active discovery session with the sender
        session = self.parent.sessions.get_discovery_session(message.message["node_id"])
        if not session:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: node does not have active discovery session with the sender")
            return None
        #verify the message hash 
        buff = message.message
        msg_signature = buff.pop('signature')
        #verify the message hash
        msg_data = json.dumps(buff)
        #get the public key of the sender from the session
        pk = session["pk"]
        #decrypt the message
        try:
            decrypted_data = EncryptionModule.decrypt(message.message["message"],self.parent.sk)
            
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error decrypting and parsing data : {e}")
            return None
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(pk)) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #parse the message
        decrypted_data = json.loads(decrypted_data)
        #check if the message counter is valid
        if decrypted_data["counter"] <= session["counter"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: counter not valid")
            return None
        
        #validate the message
        message.message["message"] = decrypted_data
        try :
            message=ApprovalResponseMessage(message.message)
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error validating message : {e}")
            return None
        
        
        #decrypt the test message
        try:
            decrypted_test = EncryptionModule.decrypt_symmetric(decrypted_data["data"]["test_message"],session["key"])
            if decrypted_test != "server_test":
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: test message not decrypted")
                return None
        except Exception as e:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: error decrypting test message : {e}")
            return None
        
        #get the session id
        session_id = decrypted_data["data"]["session_id"]
        #update the session
        session_data = {
            "approved": True,
            "status": "active",
        }
        self.parent.sessions.update_connection_session(session_id,session_data)
        
########################################
# Heartbeat protocol
########################################

class HeartbeatProtocol:
    
    def __init__(self,parent):
        self.parent = parent
        #define heartbeat interval
        self.heartbeat_interval = 5
        #define last heartbeat
        self.last_call = mktime(datetime.datetime.now().timetuple())
    
    def cron(self):
        #send heartbeat to all nodes
        for session_id, session in self.parent.sessions.connection_sessions.items():
            #check if time interval is passed
            session_time = mktime(datetime.datetime.now().timetuple()) - session["last_heartbeat"]
            if session_time > self.heartbeat_interval and session["status"] == "active":
                #send heartbeat
                self.send_heartbeat(session)
                #update last heartbeat time
                self.parent.sessions.connection_sessions[session_id]["last_heartbeat"] = mktime(datetime.datetime.now().timetuple())
    def handle(self,message):
        
        if message.message["type"] == "heartbeat_request":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting handle_heartbeat")
            self.handle_heartbeat(message)
        elif message.message["type"] == "heartbeat_response":
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {message.message['node_id']} of type {message.message['type']}, starting handle_heartbeat_response")
            self.handle_heartbeat_response(message)
        else:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: unknown message type {message.message['type']}")
                
    def send_heartbeat(self,session):
        
        #send heartbeat to session
        #prepare message 
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": session["counter"]+1,
                "data":self.parent.sessions.get_node_state_table(),
                "blockchain_status":self.parent.blockchain.get_sync_info()
            })
        #serialize message
        msg_data= json.dumps(msg_data)
        #encrypt message
        encrypted_msg = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
        #create heartbeat message
        payload = OrderedDict({
            "session_id": session["session_id"],
            "node_id": self.parent.node_id,
            "node_type": self.parent.node_type,
            "port": self.parent.port,
            "type": "heartbeat_request",
            "time":mktime(datetime.datetime.now().timetuple()),
            "pos": self.parent.pos,
            "message":encrypted_msg
            })
        #serialize message
        msg_data= json.dumps(payload)
        #get message hash and signature
        msg_signature = EncryptionModule.sign(msg_data,self.parent.sk)
        #add hash and signature to message
        payload["signature"] = msg_signature
        #send message
        self.parent.queues.put_queue({"target": session["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload,
                                      "pos": self.parent.pos},"outgoing")
           
    def handle_heartbeat(self,message):
        #receive heartbeat from node
        #get session
        session = self.parent.sessions.get_connection_sessions(message.message["session_id"])
        if not session:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid session")
            return
        #get message hash and signature
        buff = message.message.copy()
        msg_signature = buff.pop("signature")
        #serialize message buffer
        msg_data= json.dumps(buff)
        #verify message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid signature")
            return
        #decrypt message
        try:
            decrypted_msg = EncryptionModule.decrypt_symmetric(message.message["message"],session["key"])
        except:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid key")
            return
        #validate message
        message.message["message"] = json.loads(decrypted_msg)
        #check counter
        if message.message["message"]["counter"]<=session["counter"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid counter")
            return
        #update node state table
        #self.parent.server.logger.warning(f'table request : {json.dumps(message.message["message"]["data"])}' )
        self.parent.sessions.update_node_state_table(message.message["message"]["data"])
        #chcek blockchain status
        if self.parent.blockchain.check_sync(*message.message["message"]["blockchain_status"]) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Un synced blockchain, sending sync request")
            self.parent.blockchain.send_sync_request()
            
        #prepare message 
        msg_data = OrderedDict({
                "timestamp": str(datetime.datetime.now()),
                "counter": session["counter"]+1,
                "data":self.parent.sessions.get_node_state_table(),
                "blockchain_status":self.parent.blockchain.get_sync_info()
            })
        #serialize message
        msg_data= json.dumps(msg_data)
        
        #encrypt message
        encrypted_msg = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
        #create heartbeat message
        payload = OrderedDict({
            "session_id": session["session_id"],
            "node_id": self.parent.node_id,
            "node_type":self.parent.node_type,
            "port": self.parent.port,
            "type": "heartbeat_response",
            "time":mktime(datetime.datetime.now().timetuple()),
            "pos": self.parent.pos,
            "message":encrypted_msg
            })
        #serialize message
        msg_data= json.dumps(payload)
        #get message hash and signature
        msg_signature = EncryptionModule.sign(msg_data,self.parent.sk)
        #add hash and signature to message
        payload["signature"] = msg_signature
        #send message
        self.parent.queues.put_queue({"target": session["node_id"],
                                      "time":mktime(datetime.datetime.now().timetuple()),
                                      "message": payload,
                                      "pos": self.parent.pos},"outgoing")
 
    def handle_heartbeat_response(self,message):
        #receive heartbeat from node
        #get session
        session = self.parent.sessions.get_connection_sessions(message.message["session_id"])
        if not session:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid session")
            return
        
        #get message hash and signature
        buff = message.message.copy()
        msg_signature = buff.pop("signature")
        #serialize message buffer
        msg_data= json.dumps(buff)
        #verify message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid signature")
            return
        #decrypt message
        try:
            decrypted_msg = EncryptionModule.decrypt_symmetric(message.message["message"],session["key"])
        except:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid key")
            return
        #validate message
        message.message["message"] = json.loads(decrypted_msg)
        #self.parent.server.logger.warning(f'table response : {json.dumps(message.message["message"]["data"])}' )
        #check counter
        if message.message["message"]["counter"]<=session["counter"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid counter")
            return
        #update node state table
        self.parent.sessions.update_node_state_table(message.message["message"]["data"])
        #update session
        self.parent.sessions.update_connection_session(message.message["session_id"],{
            "counter":message.message["message"]["counter"],
            "last_active": mktime(datetime.datetime.now().timetuple())})   
        #chcek blockchain status
        if self.parent.blockchain.check_sync(*message.message["message"]["blockchain_status"]) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Un synced blockchain, sending sync request")
            self.parent.blockchain.send_sync_request()    

#######################################
# Consensus protocol SPFT
#######################################

class SBFT:
    def __init__(self,parent) -> None:
        #define parent
        self.parent = parent
        #define views
        self.views = {}
        #define view timeout
        self.view_timeout = 10
        
    def cron(self):
        #TODO implement cron for view timeout
        #check views for timeout
        for view_id,view in self.views.copy().items():
            if mktime(datetime.datetime.now().timetuple()) - view['last_updated'] > self.view_timeout:
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: View {view_id} timed out")
                self.views.pop(view_id)
        
    def handle(self, msg):
        #handle message
        msg = msg["message"]["data"]
        operation = msg['operation']
        if operation == 'pre-prepare':
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting pre-prepare")
            self.pre_prepare(msg)
        elif operation == 'prepare':
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting prepare")
            self.prepare(msg)
        elif operation == 'prepare-collect':
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting prepare-collect")
            self.prepare_collect(msg)
        elif operation == 'commit':
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting commit")
            self.commit(msg)
        elif operation == 'commit-collect':
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting commit-collect")
            self.commit_collect(msg)
        elif operation == 'sync_request':
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting sync_request")
            self.parent.blockchain.handle_sync_request(msg)
        elif operation == 'sync_reply':
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['source']} of type {msg['operation']}, starting sync_response")
            self.parent.blockchain.handle_sync_reply(msg)
        else:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Received message from {msg['message']['node_id']} of type {msg['message']['type']}, but no handler found")
            pass
    
    def send(self,msg):
        #check message type 
        if not type(msg['message']) in [dict,str]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid message type")
            return
        #create view number 
        view_id = self.generate_view_id()
        #get node_ids 
        node_ids = self.parent.sessions.get_node_state_table()
        #create view
        self.views[view_id] = {
            "timestamp":msg['timestamp'],
            "last_updated":mktime(datetime.datetime.now().timetuple()),
            "source": self.parent.node_id,
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
        msg_signature = EncryptionModule.sign(msg_data,self.parent.sk)
        #add signature to message
        msg["signature"] = msg_signature
        
        #broadcast message to the network
        self.parent.network.send_message('all',msg)
    
    def pre_prepare(self,msg):
        #handle pre-prepare message
        #check if view exists
        view_id = msg['view_id']
        if view_id in self.views.keys():
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: View is already created")
            return
        #get the session 
        session = self.parent.sessions.get_connection_session_by_node_id(msg['source'])
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)        
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #compare node state table
        if not self.parent.sessions.compare_node_state_table(msg['node_ids']):
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Node state table not equal")
            return None
        #message payload
        payload = {
            "timestamp":mktime(datetime.datetime.now().timetuple()),
            "operation":"prepare",
            "source":self.parent.node_id,
            "view_id":view_id,
            "message":msg['message']
        }
        #get hash and sign of message
        msg_data = json.dumps(payload)
        msg_signature = EncryptionModule.sign(msg_data,self.parent.sk)
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
        self.parent.network.send_message(msg['source'],payload)
    
    def prepare(self,msg):
        #handle prepare message
        #check if view exists
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #rospy.loginfo(view)
        #get session
        session = self.parent.sessions.get_connection_session_by_node_id(msg['source'])
        #check if node_id is not the source
        #rospy.loginfo(session)
        if self.parent.node_id == msg['source']:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        msg_hash = msg.pop('hash')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #check hash of message
        if msg_hash != view["hash"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Hash of message does not match")
            return None
        msg["signature"] = msg_signature
        msg["hash"] = msg_hash
        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(msg['node_ids']):
        #    if self.parent.DEBUG:
        #        rospy.loginfo("Node state table not equal")
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
            "source":self.parent.node_id,
            "prepare":self.views[view_id]["prepare"],
            "hash":view["hash"]
        }
        #get hash and sign of message
        msg_data = json.dumps(payload)
        msg_hash = EncryptionModule.hash(msg_data)
        msg_signature = EncryptionModule.sign_hash(msg_hash,self.parent.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "prepare"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        #broadcast message
        self.parent.network.send_message('all',payload)
        
    
    def prepare_collect(self,msg):
        #handle prepare-collect message
        #check if view exists
        #rospy.loginfo(msg)
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #get session
        session = self.parent.sessions.get_connection_session_by_node_id(msg['source'])
        #check if node_id is not the source
        if self.parent.node_id == msg['source']:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #check hash of message
        if msg["hash"] != view["hash"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Hash of message does not match")
            return None
        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(msg['node_ids']):
        #    if self.parent.DEBUG:
        #        rospy.loginfo("Node state table not equal")
        #    return None
        #loop in prepare-collect
        for m in msg["prepare"]:
            #verify signature
            m_signature = m.pop('signature')
            m_hash = m.pop('hash')
            m_data = json.dumps(m)
            #verify the message signature
            if EncryptionModule.verify(m_data, m_signature, EncryptionModule.reformat_public_key(view["node_ids"][m["source"]])) == False:
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: signature not verified")
                return None
            #check hash of message
            if m_hash != view["hash"]:
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: Hash of message does not match")
                return None
        #send commit message to source node
        payload = {
            "timestamp":mktime(datetime.datetime.now().timetuple()),
            "operation":"commit",
            "view_id":view_id,
            "source":self.parent.node_id,
            "hash":view["hash"]
        }
        #get hash and sign of message
        msg_data = json.dumps(payload)
        msg_hash = EncryptionModule.hash(msg_data)
        msg_signature = EncryptionModule.sign_hash(msg_hash,self.parent.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "commit"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        self.parent.network.send_message(view["source"],payload)
    def commit(self,msg):
        #handle commit message
        #check if view exists
        #rospy.loginfo(msg)
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #get session
        session = self.parent.sessions.get_connection_session_by_node_id(msg['source'])
        #check if node_id is not the source
        if self.parent.node_id == msg['source']:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None
        #check hash of message
        if msg["hash"]  != view["hash"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Hash of message does not match")
            return None
        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(view['node_ids']):
        #    if self.parent.DEBUG:
        #        rospy.loginfo("Node state table not equal")
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
            "source":self.parent.node_id,
            "commit":self.views[view_id]["commit"]
        }
        #get hash and sign of message
        msg_data = json.dumps(payload)
        msg_hash = EncryptionModule.hash(msg_data)
        msg_signature = EncryptionModule.sign_hash(msg_hash,self.parent.sk)
        #add signature to message
        payload["signature"] = msg_signature
        #update view
        self.views[view_id]["status"] = "complete"
        self.views[view_id]["last_updated"] = mktime(datetime.datetime.now().timetuple())
        #push message to output queue
        try:
            self.parent.queues.put_output_queue(view["message"],view["source"],"dict",view["timestamp"])
        except Exception as e:
            print(f"{self.parent.node_id}: ERROR : {e}")
        #broadcast message
        self.parent.network.send_message('all',payload)
    
    def commit_collect(self,msg):
        #handle commit-collect message
        #check if view exists
        view_id = msg['view_id']
        if view_id not in self.views.keys():
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: View is not created")
            return
        #get view 
        view = self.views[view_id]
        #get session
        session = self.parent.sessions.get_connection_session_by_node_id(msg['source'])
        #check if node_id is not the source
        if self.parent.node_id == msg['source']:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Node_id is the source")
            return
        #verify signature
        msg_signature = msg.pop('signature')
        #stringify the data payload
        msg_data = json.dumps(msg)
        #verify the message signature
        if EncryptionModule.verify(msg_data, msg_signature, EncryptionModule.reformat_public_key(session["pk"])) == False:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: signature not verified")
            return None

        #compare node state table
        #if not self.parent.sessions.compare_node_state_table(msg['node_ids']):
        #    if self.parent.DEBUG:
        #        rospy.loginfo("Node state table not equal")
        #    return None
        #loop in prepare-collect
        for m in msg["commit"]:
            #verify signature
            m_signature = m.pop('signature')
            m_data = json.dumps(m)
            #verify the message signature
            if EncryptionModule.verify(m_data, m_signature, EncryptionModule.reformat_public_key(view["node_ids"][m["source"]])) == False:
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: signature not verified")
                return None
            #check hash of message
            if m["hash"] != view["hash"]:
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: Hash of message does not match")
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
    
######################################
# Network Module
######################################
    
class NetworkInterface:
    
    def __init__(self,parent):
        '''
        Initialize network interface
        '''
        #define parent
        self.parent = parent
        #define discovery interval
        self.discovery_interval = 10
        #define heartbeat interval
        self.heartbeat_interval = 5
     

    def verify_data(self,message):
        #get session
        session = self.parent.sessions.get_connection_sessions(message.message["session_id"])
        if not session:
            if self.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid session")
            return

        #decrypt message
        try:
            decrypted_msg = EncryptionModule.decrypt_symmetric(message.message["message"],session["key"])
        except:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid key")
            return
        #validate message
        message.message["message"] = json.loads(decrypted_msg)
        #check counter
        if message.message["message"]["counter"]<session["counter"]:
            if self.parent.DEBUG:
                rospy.loginfo(f"{self.parent.node_id}: Invalid counter")
            return
        
        return message.message

    def send_message(self, target, message):
        
        #define target sessions
        if target == "all":
            node_ids = self.parent.sessions.get_active_nodes()
        elif type(target) == list:
            node_ids = target
        else:
            node_ids = [target]
        #iterate over target sessions
        for node_id in node_ids:
            #check if node_id is local node_id 
            if node_id == self.parent.node_id:
                continue
            #check if session is available
            if not self.parent.sessions.has_active_connection_session(node_id):
                if self.parent.DEBUG:
                    rospy.loginfo(f"{self.parent.node_id}: No active session")
                #return Response("No active session", status=400)
            #get session
            session = self.parent.sessions.get_connection_session_by_node_id(node_id)
            #prepare message data
            msg_data = OrderedDict({
            "timestamp": str(datetime.datetime.now()),
                "counter": self.parent.comm.counter,
                "data":message
                })
            #stringify message data
            msg_data = json.dumps(msg_data)
            #encrypt message data
            encrypted_data = EncryptionModule.encrypt_symmetric(msg_data,session["key"])
            #prepare message payload
            msg_payload = OrderedDict({
                "type": "data_exchange",
                "time":mktime(datetime.datetime.now().timetuple()),
                "node_id": self.parent.node_id,
                "node_type": self.parent.node_type,
                "data": msg_data,
                "pos": self.parent.pos,
                "port": self.parent.port,
                "session_id": session["session_id"],
                "message": encrypted_data
                })
            #add message to the queue
            self.parent.queues.put_queue({
                "target": session["node_id"],
                "time":mktime(datetime.datetime.now().timetuple()),
                "message": msg_payload,
                "pos": self.parent.pos,
            },"outgoing")

#####################################
# RosChain Module
#####################################

class RosChain:
    def __init__(self,node_id,node_type,endpoint,port,secret_key,base_directory,auth=None,DEBUG=False):
        '''
        Initialize network interface
        '''
        #define ros node
        self.node = rospy.init_node("roschain", anonymous=True)
        #define is_initialized
        self.ready = False
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
        #define base directory
        self.base_directory = base_directory
        #define block size 
        self.block_size = 10
        #define block tolerance 
        self.block_tolerance = 5
        #check if key pairs is available
        rospy.loginfo(f"{self.node_id}: ROSChain:Checking if key pairs are available")
        self.pk, self.sk = EncryptionModule.load_keys(f'{self.base_directory}/{self.node_id}_pk.pem', f'{self.base_directory}/{self.node_id}_sk.pem')
        #if not, create new public and private key pair
        if self.pk == None:
            rospy.loginfo(f"{self.node_id}: ROSChain:Key pairs are not available, creating new")
            self.pk, self.sk = EncryptionModule.generate_keys()
            EncryptionModule.store_keys(f'{self.node_id}_pk.pem', f'{self.node_id}_sk.pem',self.pk,self.sk)
        #define communication module
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing communication module")
        self.comm = MQTTCommunicationModule(self.node_id,self.endpoint,self.port,self.auth,self.DEBUG)
        #define session manager
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing session manager")
        self.sessions = SessionManager(self)
        #define queue
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing queue manager")
        self.queues = QueueManager()
        #define network interface
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing network interface")
        self.network = NetworkInterface(self)
        #define heartbeat protocol
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing heartbeat protocol")
        self.heartbeat = HeartbeatProtocol(self)
        #define discovery protocol
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing discovery protocol")
        self.discovery = DiscoveryProtocol(self)
        #define blockchain
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing blockchain")
        self.blockchain = Blockchain(self)
        #define consensus protocol
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing consensus protocol")
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
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing records service")
        self.get_record_service = rospy.Service(f'get_records',GetBCRecords,lambda req: self.get_records(req))
        #define submit message service
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing submit message service")
        self.submit_message_service = rospy.Service(f'submit_message',SubmitTransaction,lambda req: self.submit_message(self,req))
        #define is_initialized service
        rospy.loginfo(f"{self.node_id}: ROSChain:Initializing is_initialized service")
        self.get_status_service = rospy.Service(f'get_status',Trigger,lambda req: self.get_status(req))
        #node is ready
        self.ready = True

    def get_status(self,args):

        if len(self.sessions.get_active_nodes()) > 0:
            tag = "CONNECTED"
        else:
            if self.ready:
                tag = "READY"
            else:
                tag = "STARTING"
        return TriggerResponse(self.ready,tag)
    
    @staticmethod           
    def submit_message(self,args):
        '''
        Send message to the given public key
        '''
        rospy.loginfo(f"{self.node_id}: Task_allocator: {self.node_id} is sending message of type {args.table_name}")
        table_name = args.table_name
        data = args.message
        msg_time = mktime(datetime.datetime.now().timetuple())
        message = {
            "table_name":table_name,
            "data":data,
            "time":msg_time
        }
        #payload 
        payload = {
            "message":message,
            "source":self.node_id,
            "timestamp":msg_time
        }
        #add message to the parent queue
        self.comm.buffer.put({
            "message":payload,
            "time":msg_time,
            "type":"consensus"   
        })
        return SubmitTransactionResponse("Success")

    def get_records(self,last_record):
        records = []
        try:
            for id in range(last_record.last_trans_id,self.blockchain.db.get_last_id("blockchain")+1):
                meta,data = self.blockchain.get_transaction(id)
                records.append(json.dumps({
                    f"{id}":{"meta":meta,"data":data}
                }))
        except:
            records = []
        return GetBCRecordsResponse(records)
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
        #handle consensus queue
        self.handle_consensus_queue()
        #check if there is any message in comm buffer
        self.handle_communication_queue()
        #check if there is any message in output queue
        self.handle_output_queue()
        #handle all cron routines
        self.cron()
                
    def handle_communication_queue(self):
        #check if there is any message in comm buffer
        while self.comm.is_available():
            comm_buffer =self.comm.get()
            self.queues.put_queue(comm_buffer["message"],comm_buffer["type"])
        #get message from queue
        message_buffer = self.queues.pop_queue()
        
        if message_buffer:
            #check message type
            if str(message_buffer["type"]) == "incoming":
                message =Message(message_buffer["message"])                         
                if message.message["node_id"]==self.node_id:
                    return
                elif str(message.message["type"]).startswith("discovery"):
                    self.discovery.handle(message)
                elif str(message.message["type"]).startswith("heartbeat"):
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
                        rospy.loginfo(f"{self.node_id}: unknown message type {message.message['type']}")
            elif str(message_buffer["type"]) == "outgoing":
                self.comm.send(message_buffer["message"])
            elif str(message_buffer["type"]) == "consensus":
                self.consensus.send(message_buffer['message'])
            else:
                if self.DEBUG:
                    rospy.loginfo(f'{self.node_id}: unknown message type {message_buffer["type"]}')
      
    def handle_output_queue(self):
        #check if there is any message in output queue
        if self.queues.output_queue_count > 10:
                #get a list of transactions
                transactions = []
                for _ in range(self.block_size):
                    transactions.append(self.queues.pop_output_queue())
                self.blockchain.add_block(transactions)
                #if output_buffer:
                #    if type(output_buffer["message"]["data"]) == str:
                #        output_buffer["message"]["data"] = json.loads(output_buffer["message"]["data"])
                #    self.blockchain.add_transaction(output_buffer["message"]["table_name"],output_buffer["message"]["data"])
                    #try:
                        
                    #except Exception as e:
                    #    if self.DEBUG:
                    #        rospy.loginfo(e)
              
    def handle_consensus_queue(self):
        #check if there is any message in consensus queue
        if self.queues.consensus_queue_count >= self.block_size + self.block_tolerance:
            #get a list of transactions
            transactions = []
            for _ in range(self.block_size):
                transactions.append(self.queues.pop_output_queue())
            payload = {
                "message":json.dumps(transactions),
                "source":self.node_id,
                "timestamp":mktime(datetime.datetime.now().timetuple())
            }
            #add message to the parent queue
            self.comm.buffer.put({
                "message":payload,
                "time":mktime(datetime.datetime.now().timetuple()),
                "type":"consensus"   
            })
#####################################
# Main
#####################################             

if __name__ == "__main__":         
    ns = rospy.get_namespace()
    try :
        node_id= rospy.get_param(f'{ns}/roschain/node_id') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting node_id argument, and got : ", node_id)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_id")

    try :
        node_type= rospy.get_param(f'{ns}/roschain/node_type') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting node_type argument, and got : ", node_type)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_type")
    
    try :
        rabbitmq_endpoint= rospy.get_param(f'{ns}/roschain/rabbitmq_endpoint') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_endpoint argument, and got : ", rabbitmq_endpoint)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_endpoint")
    
    try :
        rabbitmq_username= rospy.get_param(f'{ns}/roschain/rabbitmq_username') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_username argument, and got : ", rabbitmq_username)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_username")
    
    try :
        rabbitmq_password= rospy.get_param(f'{ns}/roschain/rabbitmq_endpoint') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_password argument, and got : ", rabbitmq_password)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_password")
    
    try :
        rabbitmq_port= rospy.get_param(f'{ns}/roschain/rabbitmq_port') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting rabbitmq_port argument, and got : ", rabbitmq_port)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : rabbitmq_port")
    
    try :
        secret= rospy.get_param(f'{ns}/roschain/secret') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting secret argument, and got : ", secret)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : secret")
    
    try :
        base_directory= rospy.get_param(f'{ns}/roschain/base_directory') # node_name/argsname
        rospy.loginfo("ROSCHAIN: Getting base_directory argument, and got : ", base_directory)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : base_directory")
    auth = {
        "username": rabbitmq_username,
        "password":rabbitmq_password
    }
    auth = None
    
    node = RosChain(node_id,node_type,rabbitmq_endpoint,int(rabbitmq_port),secret,base_directory,auth,True)
  
    while not rospy.is_shutdown():
        #pop message from output queue
        node.loop()
    
            