#!/usr/bin/env python
import json
import datetime
from rospy import spin,loginfo,init_node,ServiceProxy,Publisher,Service,get_namespace,get_param,ROSInterruptException,Subscriber,is_shutdown,Rate
from collections import OrderedDict
from time import mktime
from database import Database
from encryption import EncryptionModule
from multirobot_sim.srv import  DatabaseQuery, DatabaseQueryRequest,FunctionCall,FunctionCallResponse
from std_msgs.msg import String
from queues import OrderedQueue
from queue import Queue

####################################
# Database module
###################################


class Database (object):
    def __init__(self,node_id):
        #self.working = False
        self.node_id = node_id
        self.query_client = ServiceProxy(f"{self.node_id}/query", DatabaseQuery)
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
        #loginfo(query)
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

class Blockchain:
    #initialize the blockchain
    def __init__(self,node_id,node_type,secret,base_dir,block_size=10, tolerance=5,DEBUG=False):
        
        #node id
        self.node_id = node_id
        #node type
        self.node_type = node_type
        #debug mode
        self.DEBUG = DEBUG
        #block size
        self.block_size = block_size
        #tolerance
        self.tolerance = tolerance
        #secret of the blockchain
        self.secret = secret
        #base directory
        self.base_dir = base_dir
        loginfo(f"{node_id}: Blockchain: Initializing")
        node = init_node("blochchain",anonymous=True)
        #define blockchain service
        self.server = Service(f"/{self.node_id}/blockchain/call",FunctionCall,self.handle_function_call)
        # define database manager
        loginfo(f"{self.node_id}: Blockchain:Initializing database proxy")
        self.db = Database(self.node_id)
        loginfo(f"{node_id}: blockchain: Initializing publisher & subscriber")
        #init network publisher
        self.prepare_message = Publisher(f"/{self.node_id}/network/prepare_message",String,queue_size=10)
        #init sync handler subscriper
        self.subscriber = Subscriber(f"/{self.node_id}/blockchain/blockchain_handler",String,self.handle_blockchain)
        #define connector log publisher
        self.log_publisher = Publisher(f"/{self.node_id}/connector/send_log", String, queue_size=10)
        #init sessions
        loginfo(f"{self.node_id}: Blockchain:Initializing database proxy")
        self.sessions = ServiceProxy(f"/{self.node_id}/sessions/call",FunctionCall,True)
        self.sessions.wait_for_service()
        # create tables
        self.create_tables()
        # define queue for storing data
        self.genesis_block()
        #sync timeout
        self.sync_timeout = 10
        #sync views
        self.views = OrderedDict()
        #queue 
        self.queue = Queue()
        #buffer 
        self.buffer = OrderedQueue(self.base_dir)
        self.buffer.load()
        loginfo(f"{self.node_id}: Blockchain:Initialized successfully")
        self.last_tx = self.get_last_committed_block()
        
        
    def get_last_committed_block(self):
        #get the last block 
        last_block = self.db.select("block",["*"],("id",'==',self.db.get_last_id("block")))
        if not last_block:
            return 0
        return last_block[0]["tx_end_id"]
        
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

    def get_last_id(self,table_name):
        return self.db.get_last_id(table_name)
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
        #check of the block exists
        block_exists = self.db.select("block", ["id", "combined_hash"], ("id", "==", 1))
        if block_exists:
            if block_exists[0]["combined_hash"] == combined_hash:
                return
            else:
                raise Exception("Genesis block is not valid")
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
    def add_transaction(self,table,item,time =mktime(datetime.datetime.now().timetuple())):
        
        #add the record to it's table
        item_id = item.pop("id")
        #remove the hash from the record
        current_hash = self.__get_current_hash(item)
        #add the transaction to the blockchain
        time_created = datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S") if type(time) == float else time
        self.db.insert("transactions",("item_id",item_id),("item_table",table),("hash",current_hash),("timecreated",time_created))
        tx_item = self.db.select("transactions",["*"],("item_id",'==',item_id),("item_table",'==',table))[0]
        
        return tx_item

    def add_block(self):
        #get all transactions between start and end id
        transactions_meta= []
        #pop out the id from the transactions
        for _ in range(self.block_size):
            transaction = self.buffer.pop()
            item = node.add_record(transaction["message"]["table_name"],transaction["message"]["data"])
            tx_meta = self.add_transaction(transaction["message"]["table_name"],item,transaction["message"]["time"])
            transactions_meta.append(tx_meta)    
        #get the start and end id
        start_tx = transactions_meta[0]["id"]
        end_tx = transactions_meta[-1]["id"]
        #remove id from the transaction
        for i in range(len(transactions_meta)):
            transactions_meta[i].pop("id")
        #get the merkle root
        root = self.__get_merkle_root(transactions_meta)
        loginfo(f"{self.node_id}: Blockchain: Adding block with merkle root {root}")
        #get last id of block
        last_block_id = self.db.get_last_id("block")
        #get previous hash
        prev_hash = self.__get_previous_hash(last_block_id)
        #combine the hashes
        combined_hash = self.__get_combined_hash(root,prev_hash)
        #
        #sending log info 
        time_now = mktime(datetime.datetime.now().timetuple())
        log_msg = f"{time_now},block,{time_now}"
        self.log_publisher.publish(log_msg)
        
        #add the transaction to the blockchain
        self.db.insert("block",("tx_start_id",start_tx),("tx_end_id",end_tx),("merkle_root",root),("combined_hash",combined_hash),("timecreated",datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
    
    def add_record(self,table,data):
        #add the record to it's table
        self.db.insert(table,*[(key,value) for key,value in data.items()])
        #get the inserted record
        item = self.db.select(table,["*"],*[(key,'==',value) for key,value in data.items()])[0]
        return item
        
    def add_entry(self,msg):
        hash = msg["hash"]
        msg = msg["data"]
        msg["data"]=json.loads(msg["data"])
        log_msg = f"{mktime(datetime.datetime.now().timetuple())},transaction,{msg['time']}"
        self.log_publisher(log_msg)
        self.buffer.put(msg,msg["time"],hash)
        if self.buffer.count() > self.block_size+ self.tolerance:
            node.add_block()
        
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
            prev_hash = EncryptionModule.hash(json.dumps(self.secret))
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
                if self.DEBUG:
                    loginfo(f"{self.node_id}: View {view_id} timed out, starting evaluation")
              
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
    
    def handle_blockchain(self,msg):
        msg = json.loads(msg.data)
        if msg["type"] == "blockchain_data":
            self.queue.put(msg)
        if msg["type"] == "sync_request":
            self.handle_sync_request(msg["message"])
        elif msg["type"] == "sync_reply":
            self.handle_sync_reply(msg["message"])
        else:
            loginfo(f"{self.node_id}: Unknown message type {msg['type']}")
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
            "source":self.node_id
        }
        #send the sync request to other nodes
        self.prepare_message.publish(json.dumps({"message":msg,"target":"all_active","type":"sync_request"}))

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
                "source":self.node_id
            }
            self.prepare_message.publish(json.dumps({"message":msg,"target":node_id,"type":"sync_reply"}))
 
    def handle_sync_reply(self,msg):
        #check if the view exists
        view_id = msg["message"]["data"]["view_id"]
        if view_id in self.views.keys():
            #if it does, then add the sync data to the view
            self.views[view_id]["sync_data"].append(msg["message"]["data"]["sync_data"])
            #check if the number of sync data is equal to the number of nodes
            if len(self.views[view_id]["sync_data"]) == len(self.make_function_call(self.sessions,"get_connection_sessions")):
                self.evaluate_sync_view(view_id)
        else:
            loginfo(f"{self.node_id}: view does not exist")

    def evaluate_sync_view(self,view_id):
        #check if the view exists
        if view_id not in self.views.keys():
            loginfo(f"{self.node_id}: view does not exist")
            return
        #check if the view is complete
        if self.views[view_id]["status"] != "pending":
            return
        #check if the number of sync data is more than half of the nodes
        active_nodes = len(self.make_function_call(self.sessions,"get_active_nodes"))
        participating_nodes = len(self.views[view_id]['sync_data'])
        print(f"number of sync data : {participating_nodes}")
        print(f"number of nodes : {active_nodes}")
        if len(self.views[view_id]["sync_data"]) < active_nodes//2:
            loginfo(f"{self.node_id}: not enough sync data")
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

if __name__ == "__main__":
    #get namespace 
    ns = get_namespace()
    try :
        node_id= get_param(f'{ns}blockchain/node_id') # node_name/argsname
        loginfo(f"Blockchain: Getting node_id argument, and got : {node_id}")
    except KeyError:
        raise ROSInterruptException("Invalid arguments : node_id")
    
    try :
        node_type= get_param(f'{ns}blockchain/node_type') # node_name/argsname
        loginfo(f"Blockchain: Getting node_type argument, and got : {node_type}")
    except KeyError:
        raise ROSInterruptException("Invalid arguments : node_type")
    
    try :
        secret = get_param(f'{ns}blockchain/secret') # node_name/argsname
        loginfo(f"Blockchain: Getting secret argument, and got : {secret}")
    except KeyError:
        raise ROSInterruptException("Invalid arguments : secret")
    
    try:
        base_dir = get_param(f'{ns}blockchain/base_dir') # node_name/argsname
        loginfo(f"Blockchain: Getting base_dir argument, and got : {base_dir}")
    except KeyError:
        raise ROSInterruptException("Invalid arguments : base_dir")
    
    
    node = Blockchain(node_id,node_type,secret,base_dir,DEBUG=True)
    #define rate 
    rate = Rate(10)
    
    #check queue 
    while not is_shutdown():
        #check if there is any message in the queue
        if node.queue.empty():
            continue
        #get the message
        node.add_entry(node.queue.get())
        rate.sleep()
        
        