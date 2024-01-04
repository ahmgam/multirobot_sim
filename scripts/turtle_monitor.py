#!/usr/bin/env python
import turtle
import rospy
import json
from std_msgs.msg import String 
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#Arguments : 
#1. robots : list of robot model names
#2. map_topic : map topic
  
def parseRobots(robots):
  #convert robot json to dictionary
  try:
    robots = robots.split(',')
    for i in range(len(robots)):
      robots[i]=robots[i].split(':')
      robots[i] = {"model_name":robots[i][0],"type":robots[i][1],"color":f"(0,{255*i//len(robots)},{255*i//len(robots)})".strip()}
  except ValueError:
    raise ValueError("Invalid robot JSON")
  return robots

def getMap(mapTopic):
  return rospy.ServiceProxy("/static_map", GetMap)()
  

def getMapMetaData(mapTopic="/map_metadata"):
  #get map metadata
  map_msg = rospy.wait_for_message(str(mapTopic), MapMetaData)
  return map_msg

def displayMap(map_msg):
  #get map metadata
  map_meta = map_msg.info
  #get map data
  map_data = map_msg.data
  #get map resolution
  map_res = map_meta.resolution
  #get map origin
  map_origin = map_meta.origin.position
  #get map width
  map_width = map_meta.width
  #get map height
  map_height = map_meta.height
  #get map origin x
  map_origin_x = map_origin.x
  #get map origin y
  map_origin_y = map_origin.y
  #get map origin x in pixels
  map_origin_x_pix = int(map_origin_x/map_res)
  #get map origin y in pixels
  map_origin_y_pix = int(map_origin_y/map_res)
  #get map width in pixels
  map_width_pix = int(map_width/map_res)
  #get map height in pixels
  map_height_pix = int(map_height/map_res)

  #create map matrix
  map_matrix = [[0 for x in range(map_width_pix)] for y in range(map_height_pix)]
  #fill map matrix
  for i in range(map_width_pix):
    for j in range(map_height_pix):
      map_matrix[i][j] = map_data[i*map_width_pix + j]

  print("start drawing map")
  screen = turtle.Screen()
  screen.bgcolor("white")
  screen.title("Map")
  screen.setup(map_width_pix, map_height_pix, 0, 0)
  turt = turtle.Turtle()
  turt.color((0,0,0))

  turt.penup()
  turt.goto(0, 0)
  turt.pendown()

  for i in range(map_width_pix):
    turt.goto(i, 0)
    turt.setheading(270)
    for j in range(map_height_pix):
      turt.forward(1)
      turt.color((map_matrix[i][j], map_matrix[i][j], map_matrix[i][j]))
      
  screen.setworldcoordinates(
    map_origin_x - map_width_pix,
    map_origin_y - map_height_pix,
    map_width_pix - map_origin_x,
    map_height_pix - map_origin_y)
  screen.tracer(0)
  print(screen )

def initializeTurtles(robots):
  #initialize turtles
  turtles = []
  for robot in robots:
    t = turtle.Turtle()
    t.name = robot['model_name']
    #t.color(robot['color'])
    if (robot['type'] == 'uav'):
      t.shape('turtle')
    elif (robot['type'] == 'ugv'):
      t.shape('square')
    else:
      t.shape('triangle')
    t.pendown()

    turtles.append(t)
  return turtles

if __name__ == '__main__':
  #getting arguments
  try :
    robots = rospy.get_param('/turtle_monitor/robots') # node_name/argsname
    print("Getting robot argument, and got : ", robots)

    map = rospy.get_param('/turtle_monitor/map')
    print("Getting map topic argument, and got : ", map)

  except rospy.ROSInterruptException:
    raise rospy.ROSInterruptException

  #initialize node
  print("Initializing node")
  rospy.init_node('turtle_monitor', anonymous=True)
  #parse robots
  print("Parsing robots")

  robots = parseRobots(robots)
  #get map message from map topic
  print("getting map")
  
  map_msg = getMap(map)
  map_msg=map_msg.map
  print(f"map type : {type(map_msg)}")
  #get map metadata
  map_meta = map_msg.info
  #display map
  print("Displaying map")
  displayMap(map_msg)
  #s = turtle.Screen()
  #s.setup(map_meta.width/map_meta.resolution, map_meta.height/map_meta.resolution, 0, 0)
  #initialize turtles
  print("Initializing turtles :", robots)
  turtles = initializeTurtles(robots)

  print("Turtles initialized")
  #subscribe to robot position topic
  print("Subscribing to robot position topic")
  model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
  rate = rospy.Rate(10) # 10hz
  # spin() simply keeps python from exiting until this node is stopped
  print("Spinning")
  while not rospy.is_shutdown():
    for t in turtles:
      try:
        resp_coordinates = model_coordinates(t.name, "world")
        (roll, pitch, yaw) =euler_from_quaternion((
          resp_coordinates.pose.orientation.x,
          resp_coordinates.pose.orientation.y,
          resp_coordinates.pose.orientation.z,
          resp_coordinates.pose.orientation.w
        ))
        t.setx(resp_coordinates.pose.position.x//map_meta.resolution)
        t.sety(resp_coordinates.pose.position.y//map_meta.resolution)
        t.setheading(yaw)
      except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))

    rate.sleep()
  rospy.spin()
  
