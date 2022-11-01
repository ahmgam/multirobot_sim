#!/usr/bin/env python3
import rospy
from math import sqrt, atan2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
goal = None

WHEEL_DIAMETER = 0.066
TOLERANCE= 0.2
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0


class PID:
    def __init__(self,kp,kd,ki):
        
        self.kp=kp
        self.kd=kd
        self.ki = ki

        self.t = 0
        self.e = 0
        self.y = 0

        self.t0 = None
        self.e0 = None
        self.y0 = None

        self.goal = None
        self.integral = 0

        self.result = 0

    def update(self,y,t):
        self.y0 = self.y
        self.y = y
        self.t0 = self.t
        self.t = t
        self.e0 = self.e
        self.e = self.y - self.goal

    def setGoal(self,goal):
        self.goal = goal

    def calculate(self):
        p = self.kp * self.e
        d = self.kd* (self.e-self.e0)/(self.t - self.t0)
        self.integral = self.ki * self.e * (self.t - self.t0) + self.integral
        self.result = p + d + self.integral
        return self.result

    def log(self):
        rospy.loginfo("e: %f,t: %f, result: %f",self.e,self.t,self.result)


def getHeadingAngle(odom_msg):
    (_, _, yaw) =euler_from_quaternion((
          odom_msg.pose.pose.orientation.x,
          odom_msg.pose.pose.orientation.y,
          odom_msg.pose.pose.orientation.z,
          odom_msg.pose.pose.orientation.w
        ))
    return yaw

def getGoalHeading(odom_msg,goal):
    return atan2(goal.pose.position.y - odom_msg.pose.pose.position.y,goal.pose.position.x - odom_msg.pose.pose.position.x)

def getDistance(odom_msg,goal):
    return sqrt(
        (odom_msg.pose.pose.position.x - goal.pose.position.x )**2 +
        (odom_msg.pose.pose.position.y - goal.pose.position.y )**2
    )
    
def is_reached(odom_msg, goal):
    return getDistance(odom_msg, goal) < TOLERANCE

def calculateTwist(angleCtl, distanceCtl):
    twist = Twist()
    twist.linear.x = min(abs(distanceCtl), MAX_LINEAR_SPEED)
    twist.angular.z = -min(abs(angleCtl), MAX_ANGULAR_SPEED)
    return twist


def callback(msg):
    global goal
    goal = msg
    
def getOdomMsg(odom_topic):
    return rospy.wait_for_message(str(odom_topic), Odometry)

if __name__ == '__main__':
    #getting arguments
    ns = rospy.get_namespace()

    try :
        odom_topic= rospy.get_param(f'{ns}/simple_controller/odom_topic') # node_name/argsname
        rospy.loginfo("simple_controller:Getting robot argument, and got : ", odom_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : odom_topic")

    try :
        cmd_topic = rospy.get_param(f'{ns}/simple_controller/cmd_topic') # node_name/argsname
        rospy.loginfo("simple_controller:Getting map topic argument, and got : ", cmd_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : cmd_topic")


    try :
        goal_topic = rospy.get_param(f'{ns}/simple_controller/goal_topic') # node_name/argsname
        rospy.loginfo("simple_controller:Getting map topic argument, and got : ", goal_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : goal_topic")
    #initialize node
    rospy.loginfo("simple_controller:Initializing node")
    rospy.init_node('simple_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #create subscriber
    rospy.loginfo("simple_controller:Creating subscriber")
    #wait for topic to be published
    rospy.wait_for_message(goal_topic, PoseStamped)
    sub = rospy.Subscriber(goal_topic, PoseStamped, callback, queue_size=10)
    #create publisher
    rospy.loginfo("simple_controller:Creating publisher")
    pub = rospy.Publisher(cmd_topic, Twist, queue_size=10)
    headingController = PID(0.5,0,0)
    headingController.setGoal(0)

    distanceController = PID(1,0,0)
    distanceController.setGoal(0)
    rospy.loginfo("simple_controller:Starting loop")
    while not rospy.is_shutdown():
        #get current position
        odom_msg = getOdomMsg(odom_topic)
        if goal is not None and not is_reached(odom_msg, goal):
            #calculate twist
            heading_angle = getHeadingAngle(odom_msg)
            goal_angle = getGoalHeading(odom_msg,goal)
            difference_angle = heading_angle - goal_angle
            distance = getDistance(odom_msg,goal)

            headingController.update(difference_angle,rospy.get_time())
            distanceController.update(distance,rospy.get_time())

            angleCtrl = headingController.calculate()
            distanceCtrl = distanceController.calculate()

            #headingController.log()
            twist = calculateTwist(angleCtrl, distanceCtrl)
            #publish twist
            pub.publish(twist)
        else:
            #publish zero twist
            pub.publish(Twist())
        rate.sleep()
