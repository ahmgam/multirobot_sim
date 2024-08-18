#!/usr/bin/env python3
import rospy
import numpy as np
from math import sqrt, atan2,pi,copysign
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped,Point,Pose
from geometry_msgs.msg import Twist
from actionlib import SimpleActionServer
from nav_msgs.msg import Path
from multirobot_sim.msg import NavigationActionAction,NavigationActionActionFeedback,NavigationActionActionResult
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
                        # make the decision by compare the distance between the intersections and the next point in self.path.poses
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
                    
                    # only exit loop if the solution pt found is closer to the next pt in self.path.poses than the current pos
                    if self.getDistance (self.goal, nextPoint) < self.getDistance ([currentX, currentY], nextPoint):
                        # update self.lastFoundIndex and exit
                        self.lastFoundIndex = i
                        break
                    else:
                        # in case for some reason the robot cannot find intersection in the next self.path.poses segment, but we also don't want it to go backward
                        self.lastFoundIndex = i+1
                    
                # if no solutions are in range
                else:
                    # no new intersection found, potentially deviated from the self.path.poses
                    # follow self.path.poses[self.lastFoundIndex]
                    self.goal = [self.path[self.lastFoundIndex].x, self.path[self.lastFoundIndex].y]

            # if determinant < 0
            else:
                # no new intersection found, potentially deviated from the self.path.poses
                # follow self.path.poses[self.lastFoundIndex]
                self.goal = [self.path[self.lastFoundIndex].x, self.path[self.lastFoundIndex].y]


class Navigator:
    def __init__(self,node_id,odom_topic,goal_topic,path_topic):
        self.node_id = node_id
        self.goal_topic=goal_topic
        self.path_topic=path_topic
        self.odom_topic=odom_topic
        self.position = None
        #initialize node
        rospy.loginfo(f"{self.node_id}: navigator:Initializing node")
        rospy.init_node('navigator', anonymous=True)

        try:
            rospy.loginfo(f"{self.node_id}: navigator:Creating cmd publisher")
            self.goalPublisher = rospy.Publisher(self.goal_topic,PoseStamped, queue_size=10)
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("{controller.node_id}: navigator:Error creating cmd subscriber")

        try:
            rospy.loginfo(f"{self.node_id}: navigator:Creating path publisher")
            self.pathPublisher = rospy.Publisher(self.path_topic, Path, queue_size=10)
        except rospy.ROSInterruptException:
            raise rospy.ROSInterruptException("navigator:Error creating path publisher")
        
        try :
            rospy.loginfo(f"{self.node_id}: navigator:Getting map")
            self.map = self.getTheMap()
        except rospy.service.ServiceException:
            raise rospy.service.ServiceException(f"{controller.node_id}: navigator:Error getting map")
        #get initial position
        rospy.loginfo(f"{self.node_id}: navigator:Getting initial position")
        odom = self.getOdomMsg()
        self.tracker = Tracker()
        self.goal = None
        self.rate = rospy.Rate(20) # 10hz

        #define action server
        self.actionServer = SimpleActionServer('navigator',NavigationActionAction,execute_cb=lambda goal: self.execute(goal,self),auto_start=True)

    @staticmethod    
    def execute(path,controller):
        path = path.points
        rospy.loginfo(f"{controller.node_id}: navigator:Received new path")
        controller.tracker.flush()
        controller.tracker.setPath(path)
        controller.goal = controller.tracker.goal
        controller.navigate()
        m = NavigationActionActionResult()
        m.result.success = True
        controller.actionServer.set_succeeded(m.result)
        

    def feedback(self,goal,controller):
        f = controller.tracker.lastFoundIndex / len(controller.tracker.path) * 100
        self.actionServer.publish_feedback(f)
    def getTheMap(self,mapService='/static_map'):
        #wait for map service
        rospy.loginfo(f"{self.node_id}: navigator:Waiting for map service")
        serv = rospy.ServiceProxy(mapService, GetMap)
        serv.wait_for_service()
        map = serv().map
        return map

    def getOdomMsg(self):
        odom = rospy.wait_for_message(self.odom_topic, Odometry)
        return odom

    def feedback(self):
        f = NavigationActionActionFeedback()
        f.feedback.percentComplete = self.tracker.lastFoundIndex / len(self.tracker.path) * 100
        self.actionServer.publish_feedback(f)
        
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
    

    def is_reached(self,odom_msg, goal):
        return self.getDistance(odom_msg, goal) < TOLERANCE

    def updatePosition(self,odom_msg):
        self.position = (odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y)
    
    def parsePath(self,path):
        formattedPath = Path()
        formattedPath.header.frame_id = "map"
        for node in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.position.z = 0
            formattedPath.poses.append(pose)
        return formattedPath
    
    def publishPath(self,path):
        path = self.parsePath(path)
        self.pathPublisher.publish(path)

    def navigate(self):
        lastGoal = None
        odom_msg = self.getOdomMsg()
        while not self.is_reached(odom_msg, self.goal):
            odom_msg = self.getOdomMsg()
            self.updatePosition(odom_msg)
            #update tracker
            self.tracker.update(self.position)
            self.goal = self.tracker.goal
            #publish path
            self.publishPath(self.tracker.path)
            #publish feedback
            #self.feedback()
            #publish goal
            if self.goal != lastGoal:
                self.goalPublisher.publish(PoseStamped(pose=Pose(position=Point(x=self.goal[0],y=self.goal[1]))))
                lastGoal = self.goal
            self.rate.sleep()
        rospy.loginfo(f"{controller.node_id}: navigator:Reached goal, setting new goal")                 
        self.goal = None
        self.tracker.flush()

if __name__ == '__main__':
    #getting arguments
    ns = rospy.get_namespace()
    try :
        node_id= rospy.get_param(f'{ns}/navigator/node_id') # node_name/argsname
        rospy.loginfo("navigator:Getting node_id argument, and got : ", node_id)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : node_id")
    
    try :
        odom_topic= rospy.get_param(f'{ns}/navigator/odom_topic') # node_name/argsname
        rospy.loginfo("navigator:Getting robot argument, and got : ", odom_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : odom_topic")

    try :
        goal_topic = rospy.get_param(f'{ns}/navigator/goal_topic') # node_name/argsname
        rospy.loginfo("navigator:Getting map topic argument, and got : ", goal_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : goal_topic")

    try :
        path_topic = rospy.get_param(f'{ns}/navigator/path_topic') # node_name/argsname
        rospy.loginfo("navigator:Getting path topic argument, and got : ", path_topic)

    except rospy.ROSInterruptException:
        raise rospy.ROSInterruptException("Invalid arguments : goal_topic")

    #initialize node
    controller = Navigator(node_id,odom_topic,goal_topic,path_topic)
    rospy.spin()
