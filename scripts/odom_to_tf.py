#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf import TransformBroadcaster

#Arguments : 
#1. robots : list of robot model names
#2. map_topic : map topic
def callback(msg,frames):
    br = TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x,
    msg.pose.pose.position.y,
    msg.pose.pose.position.z),
    (msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w),
    rospy.Time.now(),
    frames["child_frame"],
    frames["parent_frame"])


if __name__ == '__main__':
  #getting arguments
  try :
    ns = rospy.get_namespace()
    topic = rospy.get_param(f'{ns}/odom_to_tf/topic') # node_name/argsname
    print("Getting topic argument, and got : ", topic)
    parent_frame = rospy.get_param(f'{ns}/odom_to_tf/parent_frame') # node_name/argsname
    print("Getting topic argument, and got : ", parent_frame)
    child_frame = rospy.get_param(f'{ns}/odom_to_tf/child_frame') # node_name/argsname
    print("Getting topic argument, and got : ", child_frame)
  except rospy.ROSInterruptException:
    raise rospy.ROSInterruptException

  # spin() simply keeps python from exiting until this node is stopped
  rospy.init_node('odom_to_tf', anonymous=True)
  rospy.Subscriber(topic,
  Odometry,
  callback,
  {"parent_frame":parent_frame, "child_frame":child_frame})
  rate = rospy.Rate(10) # 10hz
  rospy.spin()
  
  while not rospy.is_shutdown():

    pass
    rate.sleep()
  rospy.spin()
