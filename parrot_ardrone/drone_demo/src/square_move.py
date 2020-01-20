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
    self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
    self._takeoff_msg = Empty()
    self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
    self._land_msg = Empty()
    self._sub_gt_pose = rospy.Subscriber("/drone/gt_pose",Pose, self.poseCallback)
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

  # function that makes the drone to go at a specific location
  def go_to_goal(self, x_goal, y_goal, dist_tolerance):

    r = rospy.Rate(10)
    self.stop_drone()
    self.landing()
    self.take_off()
    self.stop_drone()
    self.stop_drone()
    self.stop_drone()
    velocity_message = Twist()

    # proportional controller for angular velocity (after take-off) 
    while (True):
      K_angular = 2 # 4.0
      desired_angle_goal = atan2(y_goal-self.y, x_goal-self.x)
      rospy.loginfo("Desired_angle:" + str(desired_angle_goal))
      angular_velocity = K_angular * (desired_angle_goal-self.yaw)
      velocity_message.angular.z = angular_velocity
      self.publish_once_in_cmd_vel(velocity_message)
      rospy.loginfo("Angular_velocity" + str(angular_velocity))

      if angular_velocity <= 0.05 and angular_velocity >= -0.05:
        rospy.loginfo("Reached at the goal angle")
        break
    self.stop_drone()
    self.stop_drone()
    self.stop_drone()
    self.stop_drone()
    time.sleep(1)

    # proportional controller for both angular and linear velocity (navigation)
    while (True):
      K_linear = 0.45 # 0.5
      distance = abs(sqrt(((x_goal-self.x)**2) + ((y_goal-self.y)**2)))
      linear_velocity = K_linear * distance

      K_angular = 4.0 # 4.0
      desired_angle_goal = atan2(y_goal-self.y, x_goal-self.x)
      angular_velocity = K_angular * (desired_angle_goal-self.yaw)

      velocity_message.linear.x = linear_velocity
      velocity_message.angular.z = angular_velocity

      self.publish_once_in_cmd_vel(velocity_message)

      if distance < dist_tolerance:
        rospy.loginfo("Reached at the goal point")
        break

      r.sleep()
    #self.turn_drone_l()
    #time.sleep(1)
    self.stop_drone()
    self.landing()


  def getDistance(self, x1, x2, y1, y2):
    return sqrt(pow(x1-x2),2)+pow((y1-y2),2)

  def move_square(self):
    
    # helper variables
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
    self.landing()
      
if __name__ == '__main__':
  rospy.init_node("move_in_patterns", anonymous=True)
  drone = MoveInPatterns()
  try:
      #drone.move_square()
      drone.go_to_goal(0.0, 0.0, 0.03)
      drone.go_to_goal(10, 10, 0.03)

      # What is going on???
      drone.go_to_goal(-10.0, 10.0, 0.03)

      drone.go_to_goal(-10.0, -10.0, 0.03)
      drone.go_to_goal(10.0, -10.0, 0.03)
      
           
      drone.go_to_goal(6, 6, 0.01)
  except rospy.ROSInterruptException:
      rospy.loginfo("Node terminated")
