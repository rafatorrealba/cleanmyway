#! /usr/bin/env python

import numpy
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        """************************************************************
        ** Iniciar variables
        ************************************************************"""
        self.linear_velocity = 0.5  # unit: m/s
        self.angular_velocity = 0.7  # unit: m/s
        self.scan_ranges1 = [360]  # Scan resolution is 1[deg]
        self.scan_ranges2 = [719]
        self.scan_ranges3 = [0]

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Iniciar publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Iniciar subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""

        self.detect_obstacle_timer = self.create_timer(
            0.010,  # unit: s
            self.detect_obstacle)

        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def detect_obstacle(self):
        twist = Twist()
        obstacle_distance = min(self.scan_ranges1)
        safety_distance = 0.5
        safety_distance_turn = 0.3

        if obstacle_distance > safety_distance:
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0

        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("Obstacles are detected nearby. Robot stopped.")



        self.cmd_vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)


if __name__ == '__main__':
    main()
