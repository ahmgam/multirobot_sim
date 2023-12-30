#!/usr/bin/env python
import sqlite3
from multirobot_sim.srv import DatabaseQuery, DatabaseQueryResponse
from queue import Queue
import random
import string
import rospy
import json
from time import sleep
class Database (object):
    def __init__(self, node_id,path,schema=None):
        #self.working = False
        self.node = rospy.init_node("database", anonymous=True)
        self.node_id = node_id
        self.connection = sqlite3.connect(f"{path}/{node_id}.sqlite3", check_same_thread=False)
        self.connection.row_factory = Database.dict_factory
        self.input_queue = Queue()
        self.output_queue = Queue()
        rospy.loginfo(f"{self.node_id}: Database:Initializing query service")
        self.query_service = rospy.Service(f"{node_id}/database/query", DatabaseQuery, self.query_handler)
        if schema:
            with open(schema) as f:
                self.connection.executescript(f.read())
        self.data = {}
        rospy.loginfo(f"{self.node_id}: Database:Initialized successfully")

    def query(self, query, args=()):   
     
        self.working = True
        with self.connection:
            try:
                cursor = self.connection.cursor()
                cursor.execute(query, args)
                #self.connection.commit()  
                if query.startswith('INSERT') is True or query.startswith('UPDATE') is True or query.startswith('DELETE') is True:
                    ret = cursor.lastrowid ,[]
                else:
                    ret = 0,cursor.fetchall()
            except Exception as e:
                rospy.loginfo(f"Error executing query, Query : {query} , Error : {e}")
                ret = 0,[]
        return ret
    
    def query_handler(self,req):
        op_id = self.generate_random_str()
        self.input_queue.put(op_id)
        self.data[op_id]= {"query":req.query,"result":None}
        while not self.is_ready(op_id):
            pass
        return DatabaseQueryResponse(*self.data[op_id]["result"])
    
    @staticmethod
    def dict_factory(cursor, row):
        d = {}
        for idx, col in enumerate(cursor.description):
            d[col[0]] = row[idx]
        r = json.dumps(d)
        return r
        
    def generate_random_str(self):
        return ''.join(random.choice(string.ascii_letters) for _ in range(10))
    
    def is_ready(self,op_id):
        if op_id in self.output_queue.queue:
            return True
        return False
    
    def loop(self):
        #get input request 
        while not self.input_queue.empty():
            op_id = self.input_queue.get()
            query = self.data[op_id]["query"]
            #execute query
            self.data[op_id]["result"] = self.query(query)
            #put output request 
            self.output_queue.put(op_id)
            sleep(0.1)
        return
    
if __name__ == "__main__":
    ns = rospy.get_namespace()
    try :
        node_id= rospy.get_param(f'{ns}database/node_id') # node_name/argsname
        rospy.loginfo("Database: Getting node_id argument, and got : ", node_id)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_id")

    try :
        db_dir= rospy.get_param(f'{ns}database/db_dir') # node_name/argsname
        rospy.loginfo("Database: Getting db_dir argument, and got : ", db_dir)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : db_dir")
    
    schema= rospy.get_param(f'{ns}database/schema',None) # node_name/argsname
    rospy.loginfo("Database: Getting shcema argument, and got : ", schema)

    
    
    database = Database(node_id,db_dir,schema)
    while not rospy.is_shutdown():
        database.loop()