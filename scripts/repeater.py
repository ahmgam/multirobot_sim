#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
class Repeater:
    def __init__(self,model_name,source_topic,target_topic,path_topic,rate=10,tolerance=0.5):
        self.tolerance = tolerance
        self.model_name = model_name
        self.source_topic = f"/{self.model_name}/{source_topic}"
        self.target_topic = f"/{self.model_name}/{target_topic}"
        self.path_topic = f"/{self.model_name}/{path_topic}"
        self.goalPose = None
        self.pose = None
        try:
            rospy.init_node('repeater', anonymous=True)
            self.rate = rospy.Rate(rate)
            rospy.loginfo(f"{model_name}_repeater:Node Initialized")
            self.publisher = rospy.Publisher(target_topic, PoseStamped, queue_size=10)
            rospy.loginfo(f"{model_name}_repeater:target topic publisher initialized")
            self.pathPublisher = rospy.Publisher(self.path_topic, Path, queue_size=10)
            rospy.loginfo(f"{model_name}_repeater:path publisher initialized")
            self.service_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            self.service_proxy.wait_for_service()
            rospy.loginfo(f"{model_name}_repeater:service proxy initialized")
            rospy.Subscriber(source_topic, PoseStamped, self.callback,self)
            rospy.loginfo(f"{model_name}_repeater:source topic subscriber initialized")
        except rospy.ROSInterruptException as e:
            raise rospy.ROSInterruptException(f"{model_name}_repeater:Error initializing node {self.model_name}, Error : {e}")
        
    @staticmethod
    def callback(data,repeater):
        repeater.goalPose = data
        repeater.publishPath()

    def updatePose(self):
        self.pose = self.service_proxy(self.model_name, 'world').pose

    def InTolerance(self,goal_pose, tolerance):
        return (abs(goal_pose.pose.position.x - self.pose.position.x) < tolerance) and (abs(goal_pose.pose.position.y - self.pose.position.y) < tolerance)
    
    def IsGoalReached(self,goal_pose, tolerance):
        if self.InTolerance(goal_pose, tolerance):
            return True
        else:
            return False
        
    def currentPoseStamped(self):
        pose = self.getPoseStamped(self.pose.position.x,self.pose.position.y,self.pose.position.z)
        return pose
    
    def getPoseStamped(self,x,y,z):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose
    
    def getTwoPointPath(self,x,y,z):
        path = Path()
        path.header.frame_id = "map"
        path.poses.append(self.currentPoseStamped())
        path.poses.append(self.getPoseStamped(x,y,z))
        return path
        
    def publish(self):
        if self.goalPose is not None:
            self.publisher.publish(self.goalPose)

    def publishPath(self):
        path = self.getTwoPointPath(self.goalPose.position.x,self.goalPose.position.y,self.goalPose.position.z)
        self.pathPublisher.publish(path)

    def loop(self):
        # update robot pose
        self.updatePose()
        # check if goal is reached
        if self.goalPose != None:
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
        model_name= rospy.get_param(f'{ns}repeater/model_name') # node_name/argsname
        rospy.loginfo(f"{ns}_repeater:Getting model_name argument, and got : ", model_name)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : model_name")

    try :
        source_topic = rospy.get_param(f'{ns}/repeater/source_topic') # node_name/argsname
        rospy.loginfo(f"{ns}_repeater:Getting source_topic argument, and got : ", source_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : source_topic")


    try :
        target_topic = rospy.get_param(f'{ns}/repeater/target_topic') # node_name/argsname
        rospy.loginfo(f"{ns}_repeater:Getting target_topic argument, and got : ", target_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : target_topic")

    try :
        path_topic = rospy.get_param(f'{ns}/repeater/path_topic') # node_name/argsname
        rospy.loginfo(f"{ns}_repeater:Getting path_topic argument, and got : ", path_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : path_topic")
    
    #initialize repeater 
    repeater = Repeater(model_name,source_topic,target_topic,path_topic)
    while not rospy.is_shutdown():
        repeater.loop()
