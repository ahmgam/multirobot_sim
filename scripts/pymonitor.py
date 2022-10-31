#!/usr/bin/env python3
from math import ceil
import rospy
from nav_msgs.srv import GetMap
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
import numpy as np
import pygame
from cv2 import resize,imread
import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#import poseStamped message
from geometry_msgs.msg import PoseStamped
# define the default robots sizes in meters
ROBOT_SIZE_UAV = 0.5
ROBOT_SIZE_UGV = 0.5
WORLD_SIZE = 10
MIN_FIG_SIZE = 30
TOLERANCE=0.2
ELEVATION=2
#imges locations
IMG_UGV = "/home/gemy/catkin_ws/src/multirobot_sim/resources/ugv-icon.png"
IMG_UAV = "/home/gemy/catkin_ws/src/multirobot_sim/resources/uav-icon.png"
#default goals topics
UGV_GOAL_TOPIC = "/goal"
UAV_GOAL_TOPIC = "/command/pose"
def parseStringTuple(string):
  if string.startswith('(') and string.endswith(')') and string.find(',') != -1:
    items = string[1:-1].split(',')
    if len(items) == 2:
      try:
        return (int(items[0]),int(items[1]))
      except ValueError:
        raise ValueError("Invalid tuple")
    else :
      raise ValueError("Invalid tuple")
    
def createRobot(type,scale,resolution,robot_size=None ):
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
def worldToPixel(x,y,origin_x,origin_y,resolution,scale):
    return (int(((x-origin_x)*scale[0])/resolution),int(((y-origin_y)*scale[1])/resolution))
def pixelToWorld(x,y,origin_x,origin_y,resolution,scale):
  return (x*resolution/scale[0]+origin_x,y*resolution/scale[1]+origin_y)


def inTolerance(x,y,z,goal_x,goal_y,goal_z,tolerance):
  return abs(x-goal_x)<tolerance and abs(y-goal_y)<tolerance and abs(z-goal_z)<tolerance
#Arguments : 
#1. robots : list of robot model names
#2. map_topic : map topic

def parseRobots(robots):
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
          robot['goal'] = parseStringTuple(value) 
      robots[i] = robot
  except ValueError:
    raise ValueError("Invalid robot JSON")
  return robots

def getMap(mapService='static_map'):
  return rospy.ServiceProxy(mapService, GetMap)()
  
def insideRect(point,pos,sprite):
  pointTransformed = (point[0]-pos[0],point[1]-pos[1])
  return sprite.get_rect().collidepoint(pointTransformed)
def displayMap(map_msg,scale):
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
def createGoal(x,y,z,reference_frame="map"):
  goal = PoseStamped()
  goal.header.frame_id = reference_frame
  goal.header.stamp = rospy.Time.now()
  goal.pose.position.x = x
  goal.pose.position.y = y
  goal.pose.position.z = z
  goal.pose.orientation.w = 1
  return goal
def initializeSurfaces(robots,scale,resolution):
  #initialize turtles
  surfaces = []
  for robot in robots:
    r = {}
    r['model_name'] = robot['model_name']
    r['type'] = robot['type']
    r['surface'] = createRobot(robot['type'],scale,resolution)
    r["name"] = robot['model_name']
    r["goal"] = robot.get('goal',None)
    if r["goal"] != None:
      r["goal"] = (r["goal"][0],r["goal"][1],ELEVATION if r["type"]=="uav" else 0)
    r["pos"] = None
    r["loc"]= None
    surfaces.append(r)
  return surfaces

if __name__ == '__main__':
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
    size = parseStringTuple(size)
  except:
    size = (800, 600)
    pass

  #initialize node
  print("Initializing node")
  rospy.init_node('pymonitor', anonymous=True)
  #parse robots
  print("Parsing robots")

  robots = parseRobots(robots)
  #get map message from map topic
  print("getting map")
  
  map_msg = getMap().map
  print(f"map type : {type(map_msg)}")
  #get map metadata
  map_meta = map_msg.info
  print(f"map metadata : {map_meta}")
  #get scales
  xScale = size[0]/map_meta.width
  yScale = size[1]/map_meta.height
  scale=(xScale,yScale)
  #display map
  print("Displaying map")
  bg,screen = displayMap(map_msg,scale)
  

  
  cursor = pygame.Surface((30,30))
  pygame.draw.rect(cursor,(255,0,0),(0,0,30,30))
  cursor.fill((0,255,0))
  #change cursor transparency
  cursor.set_alpha(50)
  #s = turtle.Screen()
  #s.setup(map_meta.width/map_meta.resolution, map_meta.height/map_meta.resolution, 0, 0)
  #initialize turtles
  print("Initializing turtles :", robots)
  surfaces = initializeSurfaces(robots,scale,map_meta.resolution)

  print("Turtles initialized")
  #subscribe to robot position topic
  print("Subscribing to robot position topic")
  model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  rate = rospy.Rate(10) # 10hz
  # spin() simply keeps python from exiting until this node is stopped
  print("Spinning")
  activeIndex = None
  while not rospy.is_shutdown():
    #diaply map
    screen.fill((255,255,255))
    screen.blit(bg, (0,0))
    
    #update robots coordinates
    for t in surfaces:
      try:
        resp_coordinates = model_coordinates(t["model_name"], "world")
        (roll, pitch, yaw) =euler_from_quaternion((
          resp_coordinates.pose.orientation.x,
          resp_coordinates.pose.orientation.y,
          resp_coordinates.pose.orientation.z,
          resp_coordinates.pose.orientation.w
        ))
        x=resp_coordinates.pose.position.x
        y=resp_coordinates.pose.position.y
        z=resp_coordinates.pose.position.z
        t["loc"]= (x,y,z)
        t["pos"] = worldToPixel(x,y,map_meta.origin.position.x,map_meta.origin.position.y,map_meta.resolution,scale)
        #prodcasting goal to robot is exist
        if t["goal"] is not None:
          if t["type"] == "ugv":
            topic = f'/{t["name"]}{UGV_GOAL_TOPIC}'
          if t["type"] == "uav":
            topic = f'/{t["name"]}{UAV_GOAL_TOPIC}'
          pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
          pub.publish(createGoal(*t["goal"]))  
          if inTolerance(*t["loc"],*t["goal"],TOLERANCE):
            print(f"Robot {t['name']} reached goal")
            t["goal"] = None   
    
        #print(f"robot {t['model_name']} position : {t['pos']} , real position : ({round(resp_coordinates.pose.position.x)},{round(resp_coordinates.pose.position.y)})")
        screen.blit(t["surface"],(t["pos"][0],t["pos"][1]))
      except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))
    #update cursor position
    if activeIndex is not None:
      screen.blit(cursor,(surfaces[activeIndex]["pos"][0],surfaces[activeIndex]["pos"][1]))
    else :
      screen.blit(cursor,(0,0))
    pygame.display.update()
    #get mouse events :
    for event in pygame.event.get():
      #print(event)
      if event.type == pygame.MOUSEBUTTONUP:
        print(f"button up found, and key is {event.button}")
        if event.button == 1:
          for i in range(len(surfaces)):
            #print(f"pose is {surfaces[i]['pos']}")
            #print(f"mouse is {event.pos}")
            #print(f"relative pos x is :{event.pos[0] - surfaces[i]['pos'][0]} and y is:{event.pos[1]-surfaces[i]['pos'][1]}")
            #if surfaces[i]["surface"].get_rect().collidepoint(event.pos):
            if insideRect(event.pos,surfaces[i]["pos"],surfaces[i]["surface"]):
              activeIndex = i
              print(f"robot {surfaces[activeIndex]['name']} is active")
          if activeIndex is not None:
            #prodcast goal to /ugv1/move_base_simple/goal topic
            pos = pixelToWorld(event.pos[0],event.pos[1],map_meta.origin.position.x,map_meta.origin.position.y,map_meta.resolution,scale)
            pos = (pos[0],pos[1],0 if surfaces[activeIndex]["type"] == "ugv" else ELEVATION)
            surfaces[activeIndex]["goal"] = pos
            #send goal to move base
            """""
            move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            move_base.wait_for_server(rospy.Duration(5))
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "world"
            goal.target_pose.header.stamp = rospy.Time.now()
            pos = pixelToWorld(event.pos[0],event.pos[1],map_meta.origin.position.x,map_meta.origin.position.y,map_meta.resolution,scale)
            goal.target_pose.pose.position.x =pos[0]
            goal.target_pose.pose.position.y = pos[1]
            goal.target_pose.pose.orientation.w = 1.0
            move_base.send_goal(goal)
            """
        if event.button == 3:
          activeIndex = None
        if event.button ==2:
          surfaces[activeIndex]["goal"] = None
    #prodcat goal to /ugv1/move_base_simple/goal topic
    """""
    if activeIndex is not None:
      if surfaces[activeIndex]["goal"] is not None:
        if surfaces[activeIndex]["type"] == "ugv":
          topic = f'/{surfaces[activeIndex]["name"]}/move_base_simple/goal'
        if surfaces[activeIndex]["type"] == "uav":
          topic = f'/{surfaces[activeIndex]["name"]}/command/pose'
        pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
        pub.publish(surfaces[activeIndex]["goal"])     
    """
    rate.sleep()
  rospy.spin()
  
