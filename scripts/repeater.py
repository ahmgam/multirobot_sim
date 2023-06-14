#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped

class Repeater:
    def __init__(self,model_name,source_topic,target_topic,rate=10,tolerance=0.5):
        self.tolerance = tolerance
        self.model_name = model_name
        self.source_topic = f"/{self.model_name}/{source_topic}"
        self.target_topic = f"/{self.model_name}/{target_topic}"
        self.goalPose = None
        self.pose = None
        rospy.init_node(f'{self.model_name}_repeater')
        self.rate = rospy.Rate(rate)
        self.publisher = rospy.Publisher(target_topic, PoseStamped, queue_size=10)
        self.service_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.Subscriber(source_topic, PoseStamped, self.callback,self)
        
    @staticmethod
    def callback(data,repeater):
        repeater.goalPose = data

    def updatePose(self):
        self.pose = self.service_proxy(self.model_name, 'world').pose

    def InTolerance(self,goal_pose, tolerance):
        return (abs(goal_pose.position.x - self.pose.position.x) < tolerance) and (abs(goal_pose.position.y - self.pose.position.y) < tolerance)
    
    def IsGoalReached(self,goal_pose, tolerance):
        if self.InTolerance(goal_pose, tolerance):
            return True
        else:
            return False
        
    def publish(self):
        if self.goalPose is not None:
            self.publisher.publish(self.message)
    def loop(self):
        # update robot pose
        self.updatePose()
        # check if goal is reached
        if self.IsGoalReached(self.goalPose, self.tolerance):
            self.goalPose = None
        # publish the message
        self.publish()
        # sleep
        self.rate.sleep()



if __name__ == '__main__':
        #getting arguments
    ns = rospy.get_namespace()

    try :
        model_name= rospy.get_param(f'{ns}/repeater/model_name') # node_name/argsname
        rospy.loginfo("repeater:Getting model_name argument, and got : ", model_name)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : model_name")

    try :
        source_topic = rospy.get_param(f'{ns}/repeater/source_topic') # node_name/argsname
        rospy.loginfo("repeater:Getting source_topic argument, and got : ", source_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : source_topic")


    try :
        target_topic = rospy.get_param(f'{ns}/repeater/target_topic') # node_name/argsname
        rospy.loginfo("repeater:Getting target_topic argument, and got : ", target_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : target_topic")

 
    #initialize repeater 
    repeater = Repeater(model_name,source_topic,target_topic)
    while not rospy.is_shutdown():
        repeater.loop()
