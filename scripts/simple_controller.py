#!/usr/bin/env python3
import rospy
import numpy as np
from math import sqrt, atan2,pi,copysign
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from path_planning import AStar,RTT
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

        self.goal = 0
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


class Planner:
    def __init__(self,map,start,algorithm=None):
        self.path = Path()
        self.goal=None
        self.map = map
        self.start= start
        rospy.loginfo(f"map size is {len(map.data)}")
        self.grid = self.formatGrid(self.map)
        rospy.loginfo(f"planner:Grid formatted, shape is {self.grid.shape}")
        self.gridInfo = self.formatGridInfo(map.info)
        rospy.loginfo(f"planner:Grid info formatted, info is {self.gridInfo}")
        self.algorithm = self.defineAlgorithm(algorithm)
        rospy.loginfo(f"planner:Algorithm defined, algorithm is {type(self.algorithm)}")

    def defineAlgorithm(self,algorithm):
        if algorithm is None:
            return None
        if algorithm == "AStar":
            return AStar(self.grid,self.gridInfo)
        if algorithm == "RTT":
            return RTT(self.grid,self.gridInfo)

    def formatGrid(self,grid):
        #convert grid to 2d array
        return np.array(grid.data).reshape(grid.info.height,grid.info.width)

    def formatGridInfo(self,info):
        return {
            "width":info.width,
            "height":info.height,
            "resolution":info.resolution,
            "origin":(info.origin.position.x,info.origin.position.y)
        }
    
    def posToGrid(self,pos):
        x,y = pos[0],pos[1]
        x = int((x - self.gridInfo["origin"][0])/self.gridInfo["resolution"])
        y = int((y - self.gridInfo["origin"][1])/self.gridInfo["resolution"])
        return (x,y)

    def gridToPos(self,grid):
        x,y = grid
        x = x*self.gridInfo["resolution"] + self.gridInfo["origin"][0]
        y = y*self.gridInfo["resolution"] + self.gridInfo["origin"][1]
        return (x,y)

    def parsePath(self,path):
        formattedPath = Path()
        formattedPath.header.frame_id = "map"
        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            node = self.gridToPos(node)
            pose.pose.position.x = node[0]
            pose.pose.position.y = node[1]
            pose.pose.position.z = 0
            formattedPath.poses.append(pose)
        return formattedPath

    def plan(self):
        if self.algorithm is None:
            if self.goal is None:
                return
            self.path = self.parsePath([self.goal])
        else :
            self.algorithm.setStart(self.posToGrid(self.start))
            rospy.loginfo(f"planner:Start set to {self.algorithm.start}")
            self.algorithm.setGoal(self.posToGrid(self.goal))
            rospy.loginfo(f"planner:Goal set to {self.algorithm.goal}")
            self.algorithm.plan()
            self.path = self.parsePath(self.algorithm.path)

    def setGoal(self, x, y,z):
        self.goal = (x, y,z)
        rospy.loginfo(f"planner:Goal set to {self.goal}")


class SimpleController:
    def __init__(self,odom_topic,cmd_vel_topic,goal_topic,path_topic,planningAlgorithm=None,headingCntParams=(0.5,0.1,0),linearCntParams=(1,0.1,0)):
        self.odom_topic=odom_topic
        self.cmd_vel_topic=cmd_vel_topic
        self.goal_topic=goal_topic
        self.path_topic=path_topic
        self.headingCntParams=headingCntParams
        self.linearCntParams=linearCntParams
        #initialize node
        rospy.loginfo("simple_controller:Initializing node")
        rospy.init_node('simple_controller', anonymous=True)

        try:
            rospy.loginfo("simple_controller:Creating cmd publisher")
            self.cmdPublisher = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("simple_controller:Error creating cmd subscriber")

        try:
            rospy.loginfo("simple_controller:Creating path publisher")
            self.pathPublisher = rospy.Publisher(self.path_topic, Path, queue_size=10)
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("simple_controller:Error creating path publisher")
        
        try :
            rospy.loginfo("simple_controller:Getting map")
            self.map = self.getTheMap()
        except rospy.service.ServiceException:
            raise rospy.service.ServiceException("simple_controller:Error getting map")
        #get initial position
        rospy.loginfo("simple_controller:Getting initial position")
        odom = self.getOdomMsg()
        self.planner = Planner(self.map,(odom.pose.pose.position.x,odom.pose.pose.position.y),planningAlgorithm)
        self.headingController =PID(*headingCntParams)
        self.distanceController = PID(*linearCntParams)
        self.goal = None
        self.pointer = None
        self.rate = rospy.Rate(20) # 10hz

        try:
            rospy.loginfo("simple_controller:Creating subscriber")
            self.goalSubscriber = rospy.Subscriber(self.goal_topic, PoseStamped,self.goalCallback,self)
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("simple_controller:Error creating goal subscriber")
        rospy.loginfo("simple_controller:Initialized")

    @staticmethod    
    def goalCallback(goal_msg,controller):
        if goal_msg.pose.position != controller.planner.goal:
            controller.planner.setGoal(goal_msg.pose.position.x,goal_msg.pose.position.y,0)
            controller.planner.plan()
            if len(controller.planner.path.poses) == 0:
                rospy.loginfo("simple_controller:No path found")
            else:
                rospy.loginfo("simple_controller:Path found")
                controller.pointer = 0
                controller.goal = controller.planner.path.poses[controller.pointer]

    def getTheMap(self,mapService='/static_map'):
        #wait for map service
        rospy.loginfo("simple_controller:Waiting for map service")
        serv = rospy.ServiceProxy(mapService, GetMap)
        serv.wait_for_service()
        map = serv().map
        return map

    def getOdomMsg(self):
        odom = rospy.wait_for_message(self.odom_topic, Odometry)
        return odom

    def getHeadingAngle(self,odom_msg):
        (_, _, yaw) =euler_from_quaternion((
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
            ))
        return yaw

    def getGoalHeading(self,odom_msg,goal):
        return atan2(goal.pose.position.y - odom_msg.pose.pose.position.y,goal.pose.position.x - odom_msg.pose.pose.position.x)
        
    def getDistance(self,odom_msg,goal):
        return sqrt(
            (odom_msg.pose.pose.position.x - goal.pose.position.x )**2 +
            (odom_msg.pose.pose.position.y - goal.pose.position.y )**2
        )
    
    def getDifferenceAngle(self,goal_angle,heading_angle):
        difference_angle = goal_angle - heading_angle  
        if difference_angle > pi:
            difference_angle = difference_angle - 2*pi
        elif difference_angle < -pi:
            difference_angle = difference_angle + 2*pi
        return difference_angle

    def is_reached(self,odom_msg, goal):
        return self.getDistance(odom_msg, goal) < TOLERANCE

    def calculateTwist(self,angleCtl, distanceCtl):
        twist = Twist()
        twist.linear.x = min(abs(distanceCtl), MAX_LINEAR_SPEED)
        twist.angular.z = copysign(min(abs(angleCtl), MAX_ANGULAR_SPEED),angleCtl)
        return twist

    def loop(self):
        rospy.loginfo("simple_controller:goal is " + str(self.planner.path.poses))
        odom_msg = self.getOdomMsg()
        if self.goal is not None :
            #heading ang goal angles
            heading_angle = self.getHeadingAngle(odom_msg)
            goal_angle = self.getGoalHeading(odom_msg,self.goal)
            #get difference between heading and goal
            difference_angle = self.getDifferenceAngle(goal_angle,heading_angle) 
            #get distance to goal
            distance = self.getDistance(odom_msg,self.goal)
            #update controllers
            self.headingController.update(difference_angle,rospy.get_time())
            self.distanceController.update(distance,rospy.get_time())
            #calculate control signals
            angleCtrl =self.headingController.calculate()
            distanceCtrl = self.distanceController.calculate()
            #calculate twist
            twist = self.calculateTwist(angleCtrl, distanceCtrl)
            #publish twist
            rospy.loginfo(f"simple_controller:Publishing twist {twist}")
            self.cmdPublisher.publish(twist)
            #publish path
            rospy.loginfo(f"simple_controller:Publishing path with {len(self.planner.path.poses)} poses")
            if len(self.planner.path.poses) > 0:
                self.pathPublisher.publish(self.planner.path)
            if self.is_reached(odom_msg, self.goal):
                self.pointer += 1
                if self.pointer < len(self.planner.path.poses):
                    rospy.loginfo("simple_controller:Reached goal, setting new goal")
                    self.goal = self.planner.path.poses[self.pointer]
                else:
                    self.goal = None
        else:
            #publish zero twist
            self.cmdPublisher.publish(Twist())
        self.rate.sleep()

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

    try :
        path_topic = rospy.get_param(f'{ns}/simple_controller/path_topic') # node_name/argsname
        rospy.loginfo("simple_controller:Getting map topic argument, and got : ", goal_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : goal_topic")

    #initialize node
    controller = SimpleController(odom_topic,cmd_topic,goal_topic,path_topic,"AStar")
    while not rospy.is_shutdown():
        #get current position
        controller.loop()
