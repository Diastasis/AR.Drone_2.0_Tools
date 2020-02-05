#! /usr/bin/env python

import rospy
import time
from math import sqrt, atan2
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
import tf

class MoveInPatterns(object):

  # Class Constructor
  def __init__(self):
    self.ctrl_c = False
    self.rate = rospy.Rate(10)

    # define the different publishers, subscribers and messages that will be used
    self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self._move_msg = Twist()
    self._pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
    self._takeoff_msg = Empty()
    self._pub_land = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
    self._land_msg = Empty()
    self._sub_gt_pose = rospy.Subscriber("/ardrone/gt_pose",Pose, self.poseCallback)
    self._pose_msg = Pose()

    self.x = 0
    self.y = 0
    self.yaw = 0
  
  # function that retrieve the current drone pose
  def poseCallback(self, pose_message):
    """Here the current pose of the drone is retrieved.
    The pose_message is geometry_msg type meaning 
    that the orientation is reprented in quaternions. 
    For this reason we should convert the quaternions 
    to euler angles. In this case we assume that the 
    drone navigates in 2D space since we are interested 
    only for the yaw which is responsible for rotation 
    on the z axis"""
    self.x = pose_message.position.x
    self.y = pose_message.position.y
    quaternion = (pose_message.orientation.x,pose_message.orientation.y,pose_message.orientation.z,pose_message.orientation.w)
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    self.yaw = rpy[2]

  # function that publishes cmd_vel messages
  def publish_once_in_cmd_vel(self, msg):
    """
    This is because publishing in topics sometimes fails the first time you publish.
    In continuos publishing systems there is no big deal but in systems that publish only
    once it IS very important.
    """
    while not self.ctrl_c:
        connections = self._pub_cmd_vel.get_num_connections()
        if connections > 0:
            self._pub_cmd_vel.publish(msg) 
            rospy.loginfo("Publish in cmd_vel...")
            break
        else:
            self.rate.sleep()

  # function that makes the drone move forward
  def move_forward_drone(self):
    self._move_msg.linear.x = 1.0 #1.0
    self._move_msg.angular.z = 0.0 #0.0
    self.publish_once_in_cmd_vel(self._move_msg)
    rospy.loginfo("Moving forward...")

  # function that stops the drone from any movement
  def stop_drone(self):
    self._move_msg.linear.x = 0.0
    self._move_msg.angular.z = 0.0
    self.publish_once_in_cmd_vel(self._move_msg)
    rospy.loginfo("Stopping...")
        
  # function that makes the drone turn 90 degrees
  def turn_drone_l(self):
    self._move_msg.linear.x = 0.0
    self._move_msg.angular.z = 1.0
    self.publish_once_in_cmd_vel(self._move_msg)
    rospy.loginfo("Turning left...")
    
  # function that makes the drone turn -90 degrees
  def turn_drone_r(self):
    rospy.loginfo("Turning right...")
    self._move_msg.linear.x = 0.0
    self._move_msg.angular.z = -1.0
    self.publish_once_in_cmd_vel(self._move_msg)
  
  # function that makes the drone takeoff
  def take_off(self):
    i=0
    while not i == 3:
        self._pub_takeoff.publish(self._takeoff_msg)
        rospy.loginfo('Taking off...')
        time.sleep(1)
        i += 1
  
  # function that makes the drone to land
  def landing(self):
    i=0
    while not i == 3:
        self._pub_land.publish(self._land_msg)
        rospy.loginfo('Landing...')
        time.sleep(1)
        i += 1

  # function that makes the drone fly in a square shape
  def move_square(self):
    r = rospy.Rate(1)
    self.take_off()
    
    # define the seconds to move in each side of the square (which is taken from the goal) and the seconds to turn
    sideSeconds = 2   # 2
    turnSeconds = 1.8 # 1.8
    
    # Logic that makes the robot move forward and turn
    for i in range(0, 4):
      self.move_forward_drone()
      time.sleep(sideSeconds)
      self.turn_drone_r()
      time.sleep(turnSeconds)
      # the sequence is computed at 1 Hz frequency
      r.sleep()
    
    self.stop_drone()
    self.stop_drone()
    self.stop_drone()

      
if __name__ == '__main__':
  rospy.init_node("move_in_patterns", anonymous=True)
  drone = MoveInPatterns()
  try:
      drone.landing()
      drone.take_off()
      drone.move_square()
      drone.landing()

  except rospy.ROSInterruptException:
      rospy.loginfo("Node terminated")
