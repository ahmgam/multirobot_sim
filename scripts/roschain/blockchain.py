import json
import datetime
import rospy
from collections import OrderedDict
from time import mktime
from database import Database
from encryption import EncryptionModule
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
        time_created = datetime.datetime.fromtimestamp(time).strftime("%Y-%m-%d %H:%M:%S") if type(time) == float else time
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

