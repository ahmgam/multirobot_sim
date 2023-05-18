#!/usr/bin/env python3
from math import ceil
import rospy
from nav_msgs.srv import GetMap
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
import numpy as np
import pygame
from cv2 import resize,imread
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
# define the default robots sizes in meters
ROBOT_SIZE_UAV = 0.5
ROBOT_SIZE_UGV = 0.5
WORLD_SIZE = 10
MIN_FIG_SIZE = 30
TOLERANCE=0.2
ELEVATION=2
#imges locations
IMG_UGV = "/home/gemy/robot_ws/src/multirobot_sim/resources/ugv-icon.png"
IMG_UAV = "/home/gemy/robot_ws/src/multirobot_sim/resources/uav-icon.png"
#default goals topics
UGV_GOAL_TOPIC = "/goal"
UAV_GOAL_TOPIC = "/command/pose"
UGV_PATH_TOPIC = "/path"
UAV_PATH_TOPIC = "/command/trajectory"

class PyMonitor:
  def __init__(self):
    #get parameters
    self.robots,self.map,self.size = self.getParameters()
    #initialize node
    rospy.loginfo("PyMonitor: Initializing node")
    rospy.init_node('pymonitor', anonymous=True)
    #parse robots
    rospy.loginfo("PyMonitor: Parsing robots")
    self.robots=self.parseRobots(self.robots)
    #get map message from map topic
    rospy.loginfo("PyMonitor: getting map")
    self.map_msg = self.getTheMap()
    #get map metadata
    map_meta = self.map_msg.info
    #get scales
    xScale = self.size[0]/map_meta.width
    yScale = self.size[1]/map_meta.height
    self.scale=(xScale,yScale)
    #display map
    rospy.loginfo("PyMonitor: Displaying map")
    self.bg,self.screen = self.displayMap(self.map_msg,self.scale)
    #define cursor
    self.cursor = pygame.Surface((30,30))
    pygame.draw.rect(self.cursor,(255,0,0),(0,0,30,30))
    self.cursor.fill((0,255,0))
    #change cursor transparency
    self.cursor.set_alpha(50)
    #initialize turtles
    rospy.loginfo("PyMonitor: Initializing Robots ")
    self.surfaces = self.initializeSurfaces(self.robots,self.scale,map_meta.resolution)
    #subscribe to robot path topic
    for surface in self.surfaces:
        if surface['type'] == 'uav':
            rospy.Subscriber(f"{surface['name']}/{UAV_PATH_TOPIC}", Path, self.pathCallback,(self,surface))
        elif surface['type'] == 'ugv':
            rospy.Subscriber(f"{surface['name']}/{UGV_PATH_TOPIC}", Path, self.pathCallback,(self,surface))
        else:
            raise ValueError("Invalid robot type")
    rospy.loginfo("PyMonitor: Robots initialized")
    #subscribe to robot position topic
    rospy.loginfo("PyMonitor: Subscribing to robot position topic")
    self.model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    self.rate = rospy.Rate(10) # 10hz
    # spin() simply keeps python from exiting until this node is stopped
    rospy.loginfo("PyMonitor: Spinning")
    self.activeIndex = None

  @staticmethod
  def pathCallback(msg,args):
    self = args[0]
    surface = args[1]
    #get surface index
    index = self.surfaces.index(surface)
    #get path
    path = msg.poses
    #get path points
    points = [pose.pose.position for pose in path]
    #update surface path
    self.surfaces[index]['path'] = points
  
  def getParameters(self):
    #getting arguments
    try :
      robots = rospy.get_param('/pymonitor/robots') # node_name/argsname
      print("Getting robot argument, and got : ", robots)

    except rospy.ROSInterruptException:
      raise rospy.ROSInterruptException("Invalid arguments : Robots")
    try :
  

      map = rospy.get_param('/pymonitor/map')
      print("Getting map topic argument, and got : ", map)

    except rospy.ROSInterruptException:
      raise rospy.ROSInterruptException("Invalid arguments : Map")

    try :
      size = rospy.get_param('/pymonitor/size') # node_name/argsname
      print("Getting size argument, and got : ", size)
      size = self.parseStringTuple(size)
    except:
      size = (800, 600)
    return robots,map,size
  
  def parseStringTuple(self,string):
    if string.startswith('(') and string.endswith(')') and string.find(',') != -1:
      items = string[1:-1].split(',')
      if len(items) == 2:
        try:
          return (int(items[0]),int(items[1]))
        except ValueError:
          raise ValueError("Invalid tuple")
      else :
        raise ValueError("Invalid tuple")
    
  def createRobot(self,type,scale,resolution,robot_size=None ):
    if (type == 'uav'):
        img = imread(IMG_UAV)
        robot_size = ROBOT_SIZE_UAV if robot_size==None else robot_size
    elif (type == 'ugv'):
        img = imread(IMG_UGV)
        robot_size = ROBOT_SIZE_UGV if robot_size==None else robot_size
    else : 
        raise ValueError("Invalid robot type")
    scaledX = ceil(robot_size*scale[0]/resolution)
    scaledY = ceil(robot_size*scale[1]/resolution)
    if (scaledX < MIN_FIG_SIZE):
        scaledX = MIN_FIG_SIZE
    if (scaledY < MIN_FIG_SIZE):
        scaledY = MIN_FIG_SIZE
    #scaledSize = (int(robot_size*scale[0]), int(robot_size*scale[1]))
    resized = resize(img, (scaledX,scaledY))   
    return pygame.surfarray.make_surface(resized)

  def worldToPixel(self,x,y,origin_x,origin_y,resolution,scale):
    return (int(((x-origin_x)*scale[0])/resolution),int(((y-origin_y)*scale[1])/resolution))

  def pixelToWorld(self,x,y,origin_x,origin_y,resolution,scale):
    return (x*resolution/scale[0]+origin_x,y*resolution/scale[1]+origin_y)

  def inTolerance(self,x,y,z,goal_x,goal_y,goal_z,tolerance):
    return abs(x-goal_x)<tolerance and abs(y-goal_y)<tolerance and abs(z-goal_z)<tolerance

  def parseRobots(self,robots):
    #convert robot json to dictionary
    try:
      robots = robots.split('|')
      for i in range(len(robots)):
        properties = robots[i].split(',,')
        robot = {}
        for prop in properties:
          key,value=prop.split(':')
          if key=="name":
            robot['model_name'] = value
          if key=="type":
            robot['type'] = value
          if key=="pos":
            robot['goal'] = self.parseStringTuple(value) 
        robots[i] = robot
    except ValueError:
      raise ValueError("Invalid robot JSON")
    return robots

  def getTheMap(self,mapService='/static_map'):
    return rospy.ServiceProxy(mapService, GetMap)().map
  
  def insideRect(self,point,pos,sprite):
    pointTransformed = (point[0]-pos[0],point[1]-pos[1])
    return sprite.get_rect().collidepoint(pointTransformed)

  def displayMap(self,map_msg,scale):
    #get map metadata
    map_meta = map_msg.info
    #get map data
    map_data = map_msg.data
    #get map width
    map_width = map_meta.width
    #get map height
    map_height = map_meta.height
    print(f"map width : {map_width} , map height : {map_height}")
    scaledSize = (ceil(map_height*scale[0]), ceil(map_width*scale[1]))
    map_matrix = np.array(map_data).reshape(map_height,map_width,1)*2.55
    print(map_matrix.max())
    map_matrix=map_matrix.astype(np.uint8)
    map_matrix= np.abs(map_matrix - 255)
    map_matrix = np.concatenate((map_matrix,map_matrix,map_matrix),axis=2)
    resized = resize(map_matrix, (scaledSize[1],scaledSize[0]))
    print(f"screen size : {scaledSize}")
    surf = pygame.surfarray.make_surface(resized)
    DISPLAYSURF = pygame.display.set_mode(scaledSize)
    return surf,DISPLAYSURF

  def createGoal(self,x,y,z,reference_frame="map"):
    goal = PoseStamped()
    goal.header.frame_id = reference_frame
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    goal.pose.orientation.w = 1
    return goal

  def initializeSurfaces(self,robots,scale,resolution):
    #initialize turtles
    surfaces = []
    for robot in robots:
      r = {}
      r['model_name'] = robot['model_name']
      r['type'] = robot['type']
      r['surface'] = self.createRobot(robot['type'],scale,resolution)
      r["name"] = robot['model_name']
      r["goal"] = robot.get('goal',None)
      if r["goal"] != None:
        r["goal"] = (r["goal"][0],r["goal"][1],ELEVATION if r["type"]=="uav" else 0)
      r["pos"] = None
      r["loc"]= None
      r["path"] = None
      surfaces.append(r)
    return surfaces
  
  def rendreBackground(self):
    self.screen.fill((255,255,255))
    self.screen.blit(self.bg, (0,0))

  def updateRobotCoordinates(self,robot):
    pass
    resp_coordinates = self.model_coordinates(robot["model_name"], "world")
    (roll, pitch, yaw) =euler_from_quaternion((
      resp_coordinates.pose.orientation.x,
      resp_coordinates.pose.orientation.y,
      resp_coordinates.pose.orientation.z,
      resp_coordinates.pose.orientation.w
      ))
    x=resp_coordinates.pose.position.x
    y=resp_coordinates.pose.position.y
    z=resp_coordinates.pose.position.z
    return (x,y,z),self.worldToPixel(x,y,self.map_msg.info.origin.position.x,self.map_msg.info.origin.position.y,self.map_msg.info.resolution,self.scale)
  
  def publishGoal(self,robot):
    if robot["goal"] is not None:
      if robot["type"] == "ugv":
        topic = f'/{robot["name"]}{UGV_GOAL_TOPIC}'
      if robot["type"] == "uav":
        topic = f'/{robot["name"]}{UAV_GOAL_TOPIC}'
      pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
      pub.publish(self.createGoal(*robot["goal"]))  

  def watchForGoal(self,robot):
    if robot["goal"] is not None and self.inTolerance(*robot["loc"],*robot["goal"],TOLERANCE):
      rospy.loginfo(f"PyMonitor: Robot {robot['name']} reached goal")
      return None
    return robot["goal"]

  def displayPath(self,surface):
    if surface["path"] is not None and len(surface["path"])>0 and surface["goal"] is not None:
      for pose in surface["path"]:
        x = pose.x
        y = pose.y
        point = self.worldToPixel(x,y,self.map_msg.info.origin.position.x,self.map_msg.info.origin.position.y,self.map_msg.info.resolution,self.scale)
        pygame.draw.circle(self.screen, (0,255,0), point, 3)

  def rendreGoal(self,robot):
    if robot["goal"] is not None:
      goalLoc = robot['goal'][0],robot['goal'][1]
      goal = self.worldToPixel(*goalLoc,self.map_msg.info.origin.position.x,self.map_msg.info.origin.position.y,self.map_msg.info.resolution,self.scale)
      pygame.draw.circle(self.screen, (0,0,255), goal, 5)

  def rendreRobot(self,robot):
    if robot["loc"] is not None:
      self.screen.blit(robot["surface"],(robot["pos"][0],robot["pos"][1]))

  def renderOrigin(self):
    goal = self.worldToPixel(self.map_msg.info.origin.position.x,self.map_msg.info.origin.position.y,self.map_msg.info.origin.position.x,self.map_msg.info.origin.position.y,self.map_msg.info.resolution,self.scale)
    pygame.draw.circle(self.screen, (0,255,255), goal, 5)

  def updateScreen(self):
    #render origin
    self.renderOrigin()
    #update robots coordinates
    for t in self.surfaces:
      try:
        #update robot coordinates
        t["loc"],t["pos"] = self.updateRobotCoordinates(t)
        #prodcasting goal to robot is exist
        self.publishGoal(t)
        #check goal status
        t["goal"] = self.watchForGoal(t)  
        #rendre goal
        self.rendreGoal(t)
        #rendre path
        self.displayPath(t)
        #rendre robot
        self.rendreRobot(t)
      except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))

  def highlightSelectedRobot(self):
    if self.activeIndex is not None:
      self.screen.blit(self.cursor,(self.surfaces[self.activeIndex]["pos"][0],self.surfaces[self.activeIndex]["pos"][1]))
    else :
      self.screen.blit(self.cursor,(0,0))

  def handleMouseClick(self):
    for event in pygame.event.get():
      #print(event)
      if event.type == pygame.MOUSEBUTTONUP:
        if event.button == 1:
          for i in range(len(self.surfaces)):
            if self.insideRect(event.pos,self.surfaces[i]["pos"],self.surfaces[i]["surface"]):
              self.activeIndex = i
              print(f"robot {self.surfaces[self.activeIndex]['name']} is active")
          if self.activeIndex is not None:
            #prodcast goal to /ugv1/move_base_simple/goal topic
            pos = self.pixelToWorld(event.pos[0],event.pos[1],self.map_msg.info.origin.position.x,self.map_msg.info.origin.position.y,self.map_msg.info.resolution,self.scale)
            pos = (pos[0],pos[1],0 if self.surfaces[self.activeIndex]["type"] == "ugv" else ELEVATION)
            self.surfaces[self.activeIndex]["goal"] = pos
        if event.button == 3:
          self.activeIndex = None
        if event.button ==2:
          self.surfaces[self.activeIndex]["goal"] = None
    
  def loop(self):
    #diaply map
    self.rendreBackground()
    #update screen
    self.updateScreen()
    #update cursor position
    self.highlightSelectedRobot()
    #update screen
    pygame.display.update()
    #get mouse events :
    self.handleMouseClick()
    #sleep
    self.rate.sleep()

if __name__ == '__main__':
  monitor = PyMonitor()
  while not rospy.is_shutdown():
    monitor.loop()
  rospy.spin()
  
