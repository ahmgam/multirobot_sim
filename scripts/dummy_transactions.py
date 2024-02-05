#!/usr/bin/env python3
from multirobot_sim.srv import SubmitTransaction,SubmitTransactionRequest
from std_srvs.srv import Trigger
from rospy import ServiceProxy
import json
from rospy import ServiceProxy
from datetime import datetime
import rospy
from random import randint


class DummyTransactions:
    def __init__(self,planningAlgorithm=None):
        self.node_id,self.node_type,self.update_interval = self.getParameters()
        rospy.loginfo(f"{self.node_id}: dummy_transactions: Initializing")
        self.node = rospy.init_node('dummy_transactions', anonymous=True)
        self.pos_x = None
        self.pos_y = None
        self.last_state_update = datetime.now()
        rospy.loginfo(f"{self.node_id}: dummy_transactions: Initializing get_records service client")
        self.is_ready = ServiceProxy(f'/{self.node_id}/roschain/is_ready',Trigger)
        self.is_ready.wait_for_service(timeout=25)
        while not self.is_ready().success:
            rospy.loginfo(f"{self.node_id}: dummy_transactions: Waiting for roschain to be ready")
            rospy.sleep(5)
        self.submit_message = ServiceProxy(f'/{self.node_id}/roschain/submit_message',SubmitTransaction)
        self.submit_message.wait_for_service(timeout=25)
    
    def getParameters(self):
        rospy.loginfo(f"dummy_transactions: getting namespace")
        ns = rospy.get_namespace()
        try :
            node_id= rospy.get_param(f'{ns}/dummy_transactions/node_id') # node_name/argsname
            rospy.loginfo(f"dummy_transactions: Getting node_id argument, and got : {node_id}")

        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("Invalid arguments : node_id")

        try :
            node_type= rospy.get_param(f'{ns}/dummy_transactions/node_type') # node_name/argsname
            rospy.loginfo(f"dummy_transactions: Getting node_type argument, and got : {node_type}")

        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("Invalid arguments : node_type")
        
        try :
            update_interval= rospy.get_param(f'{ns}/dummy_transactions/update_interval',5) # node_name/argsname
            rospy.loginfo(f"dummy_transactions: Getting update_interval argument, and got : {update_interval}")

        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("Invalid arguments : update_interval")
        
        return node_id,node_type,update_interval
  
    def update_position(self):
        self.pos_x = randint(0,100)
        self.pos_y = randint(0,100)

            
    def submit_node_state(self):
        #submit node state to blockchain
        rospy.loginfo(f"{self.node_id}: dummy_transactions: Submitting node state")
        payload = {
            'node_id':self.node_id,
            'node_type':self.node_type,
            'timecreated':datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'pos_x':self.pos_x,
            'pos_y':self.pos_y,
            'details': ""
        }
        msg = SubmitTransactionRequest('states',json.dumps(payload))
        self.submit_message(msg)

    def loop(self):
        #update position
        self.update_position()

        if (datetime.now() - self.last_state_update).total_seconds() > self.update_interval:
            self.last_state_update = datetime.now()
            self.submit_node_state() 


if __name__ == "__main__":
    

    rospy.loginfo("dummy_transactions:Starting the task dummy_transactions node")
    robot = DummyTransactions()
    #define rate
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        robot.loop()
        rate.sleep()
