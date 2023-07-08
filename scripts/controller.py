#!/usr/bin/env python3
import rospy
import numpy as np
from math import sqrt, atan2,pi,copysign
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Point,Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from actionlib import SimpleActionServer,GoalStatus
from multirobot_sim.action import NavigationAction,NavigationActionGoal,NavigationActionResult,NavigationActionFeedback
goal = None

WHEEL_DIAMETER = 0.066
TOLERANCE= 0.2
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0
DISTANCE_THRESHOLD = 1.0
ANGLE_THREHOLD = 0.2
LOOKAHEAD_DIST = 0.25
class Tracker :
    def __init__(self,lookAheadDis=LOOKAHEAD_DIST):
        self.path = Path()
        self.lastFoundIndex = 0
        self.goal = None
        self.lookAheadDis = lookAheadDis
    def setPath(self,path):
        self.path = path
        self.lastFoundIndex = 0
        self.goal = self.path[self.lastFoundIndex].x,self.path[self.lastFoundIndex].y
    
    def getDistance(self,pose1,pose2):
        return sqrt((pose1[0]-pose2[0])**2 + (pose1[1]-pose2[1])**2)
    
    def flush(self):
        self.goal = None
        self.lastFoundIndex = 0
        self.path = Path()

    def update (self, currentPos) :

        rospy.loginfo("last found index : {} of {} which is {}".format(self.lastFoundIndex,len(self.path),(self.path[self.lastFoundIndex].x,self.path[self.lastFoundIndex].y)))
        # extract currentX and currentY
        currentX = currentPos[0]
        currentY = currentPos[1]

        # use for loop to search intersections
        for i in range (self.lastFoundIndex, len(self.path)-1):

            # beginning of line-circle intersection code
            x1 = self.path[i].x - currentX
            y1 = self.path[i].y - currentY
            x2 = self.path[i+1].x - currentX
            y2 = self.path[i+1].y - currentY
            dx = x2 - x1
            dy = y2 - y1
            dr = sqrt (dx**2 + dy**2)
            D = x1*y2 - x2*y1
            discriminant = (self.lookAheadDis**2) * (dr**2) - D**2

            if discriminant >= 0:
                sol_x1 = (D * dy + np.sign(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_x2 = (D * dy - np.sign(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
                sol_y2 = (- D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2

                sol_pt1 = [sol_x1 + currentX, sol_y1 + currentY]
                sol_pt2 = [sol_x2 + currentX, sol_y2 + currentY]
                # end of line-circle intersection code

                minX = min(self.path[i].x, self.path[i+1].x)
                minY = min(self.path[i].y, self.path[i+1].y)
                maxX = max(self.path[i].x, self.path[i+1].x)
                maxY = max(self.path[i].y, self.path[i+1].y)

                # if one or both of the solutions are in range
                if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):

                    nextPoint = [self.path[i+1].x,self.path[i+1].y]
                    # if both solutions are in range, check which one is better
                    if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
                        # make the decision by compare the distance between the intersections and the next point in self.path
                        if self.getDistance(sol_pt1, nextPoint) < self.getDistance(sol_pt2, nextPoint):
                            self.goal = sol_pt1
                        else:
                            self.goal = sol_pt2
                    
                    # if not both solutions are in range, take the one that's in range
                    else:
                        # if solution pt1 is in range, set that as goal point
                        if (minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY):
                            self.goal = sol_pt1
                        else:
                            self.goal = sol_pt2
                    
                    # only exit loop if the solution pt found is closer to the next pt in self.path than the current pos
                    if self.getDistance (self.goal, nextPoint) < self.getDistance ([currentX, currentY], nextPoint):
                        # update self.lastFoundIndex and exit
                        self.lastFoundIndex = i
                        break
                    else:
                        # in case for some reason the robot cannot find intersection in the next self.path segment, but we also don't want it to go backward
                        self.lastFoundIndex = i+1
                    
                # if no solutions are in range
                else:
                    # no new intersection found, potentially deviated from the self.path
                    # follow self.path[self.lastFoundIndex]
                    self.goal = [self.path[self.lastFoundIndex].x, self.path[self.lastFoundIndex].y]

            # if determinant < 0
            else:
                # no new intersection found, potentially deviated from the self.path
                # follow self.path[self.lastFoundIndex]
                self.goal = [self.path[self.lastFoundIndex].x, self.path[self.lastFoundIndex].y]

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



class SimpleController:
    def __init__(self,odom_topic,cmd_vel_topic,goal_topic,path_topic,planningAlgorithm=None,headingCntParams=(0.5,0.1,0),linearCntParams=(1,0.1,0)):
        self.odom_topic=odom_topic
        self.cmd_vel_topic=cmd_vel_topic
        self.goal_topic=goal_topic
        self.target = None
        self.path_topic=path_topic
        self.headingCntParams=headingCntParams
        self.linearCntParams=linearCntParams
        self.position = None
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
        #odom = self.getOdomMsg()
        #self.planner = Planner(self.map,(odom.pose.pose.position.x,odom.pose.pose.position.y),planningAlgorithm)
        self.headingController =PID(*headingCntParams)
        self.distanceController = PID(*linearCntParams)
        self.tracker = Tracker()
        self.goal = None
        self.rate = rospy.Rate(20) # 10hz

        try:
            rospy.loginfo("simple_controller:Creating subscriber")
            self.goalSubscriber = SimpleActionServer(
                f"{rospy.get_namespace()}/navigation",
                NavigationAction,
                execute_cb=self.goalCallback,
                auto_start=False
            )
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("simple_controller:Error creating goal subscriber")
        rospy.loginfo("simple_controller:Initialized")

    def goalCallback(self,goal_msg):
        self.tracker.flush()
        self.tracker.setPath(goal_msg)
        self.goal = [
            goal_msg[0].x,
            goal_msg[0].y
            ]
        self.target= [
            goal_msg[-1].x,
            goal_msg[-1].y
        ]
        updateMessage = NavigationActionFeedback()
        goalMessage = NavigationActionResult()
        updateMessage.percentComplete = self.tracker.lastFoundIndex/len(self.tracker.path)*100
        self.goalSubscriber.publish_feedback(updateMessage)
        if self.tracker.goal is None:
            goalMessage.success= True
            self.goalSubscriber.set_succeeded(goalMessage)

        self.loop()
        

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
        p1 = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
        #if type(goal) is PoseStamped:
        #    p2 = (goal.pose.position.x,goal.pose.position.y)
        #elif type(goal) is tuple or type(goal) is list:
        #    p2 = (goal[0],goal[1])
        #else :
            #raise TypeError(f"simple_controller:Goal type not supported {type(goal)}")
        #    p2 = (goal[0],goal[1])
        
        try : 
            p2 = (goal[0],goal[1])
        except Exception as e:
            p2 = (goal.pose.position.x,goal.pose.position.y) 
        return self.getHeading(p1,p2)
        
    def getDistance(self,odom_msg,goal):
        p1 = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
        if isinstance(goal,PoseStamped):
            p2 = (goal.pose.position.x,goal.pose.position.y)
        elif isinstance(goal,Pose):
            p2 = (goal.position.x,goal.position.y)
        elif isinstance(goal,Point):
            p2 = (goal.x,goal.y)
        else:
            p2 = (goal[0],goal[1])
        return sqrt((p1[0] - p2[0] )**2 +(p1[1] - p2[1] )**2)
    
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
    
    def updatePosition(self,odom_msg):
        self.position = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
    
    @staticmethod
    def getHeading(p1,p2):
        #return atan2(p2[1] - p1[1],p2[0] - p1[0])
    
        angle= atan2(p2[1] - p1[1],p2[0] - p1[0])
        if angle > pi:
            angle = angle - 2*pi
        elif angle < -pi:
            angle = angle + 2*pi
        return angle
        
        

    def loop(self,path):
        
        odom_msg = self.getOdomMsg()
        self.updatePosition(odom_msg)
        if self.target is not None:
            #update tracker
            self.tracker.update(self.position)
            self.goal = self.tracker.goal
            #heading ang goal angles
            heading_angle = self.getHeadingAngle(odom_msg)
            goal_angle = self.getGoalHeading(odom_msg,self.goal)
            #get difference between heading and goal
            difference_angle = self.getDifferenceAngle(goal_angle,heading_angle)
            #difference_angle = goal_angle-heading_angle 
            rospy.loginfo("difference_angle: "+str(difference_angle))
            rospy.loginfo("goal : "+str(self.goal[0])+","+str(self.goal[1]))
            rospy.loginfo("target : "+str(self.target.x)+","+str(self.target.y))
            rospy.loginfo("position : "+str(self.position[0])+","+str(self.position[1]))
            #get distance to goal
            distance = self.getDistance(odom_msg,self.goal)
            #update controllers
            self.headingController.update(difference_angle,rospy.get_time())
            self.distanceController.update(distance,rospy.get_time())
            #calculate control signals
            angleCtrl =self.headingController.calculate()
            distanceCtrl = self.distanceController.calculate()
            #condition for unreachable 
            
            if ANGLE_THREHOLD < difference_angle and distance < DISTANCE_THRESHOLD:
                distanceCtrl = 0
            
            #calculate twist
            twist = self.calculateTwist(angleCtrl, distanceCtrl)
            #publish twist
            self.cmdPublisher.publish(twist)
            #publish path
            #if len(self.planner.path.poses) > 0:
            #    self.pathPublisher.publish(self.planner.path)
            if self.is_reached(odom_msg, self.target):
                rospy.loginfo("simple_controller:Reached goal, setting new goal")                 
                self.goal = None
                self.target = None
                self.tracker.flush()
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
    rospy.spin()