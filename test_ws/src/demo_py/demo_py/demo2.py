#! /usr/bin/env python

import numpy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def scan_callback(self, msg):
      scan_ranges = msg.ranges
      init_scan_state = True

def cmd_vel_raw_callback(self, msg):
      linear_velocity = msg.linear.x
      self.angular_velocity = msg.angular.z

  def update_callback(self):
      if self.init_scan_state is True:
          self.detect_obstacle()

  def detect_obstacle(self):
      twist = Twist()
      obstacle_distance = min(self.scan_ranges)
      safety_distance = 0.3














turn_velocity = 7
linear_velocity = 0.5
qos = QoSProfile(depth=10)

def callback(msg):
    if msg.ranges[360] > 1 :
        move.linear.x = linear_velocity
        move.angular.z = 0.0

    elif msg.ranges[360] < 1 :
        move.linear.x = 0
        move.angular.z = turn_velocity
        if msg.ranges[719] < 1.5:
            move.linear.x = linear_velocity/10
            move.angular.z = -turn_velocity
        elif msg.ranges[0] < 1.5:
            move.linear.x = linear_velocity/10
            move.angular.z = turn_velocity
        else:
           move.linear.x = 0
           move.angular.z = 0

    pub.publish(move)



    # Iniciar publishers

cmd_vel_pub = create_publisher(Twist, 'cmd_vel', qos)

        # Iniciar subscribers
scan_sub = self.create_subscription(LaserScan,'scan',scan_callback, qos_profile=qos_profile_sensor_data)

cmd_vel_raw_sub = self.create_subscription(Twist,'cmd_vel_raw', cmd_vel_raw_callback, qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
update_timer = self.create_timer( 0.010,  # unit: s
update_callback)
get_logger().info("Turtlebot3 obstacle detection node has been initialised.")






move = Twist()

rospy.sleep(1000000)
