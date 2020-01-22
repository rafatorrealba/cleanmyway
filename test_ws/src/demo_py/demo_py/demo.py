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
        self.linear_velocity = 0.0  # unit: m/s
        self.angular_velocity = 0.0  # unit: m/s
        self.scan_ranges = []
        self.init_scan_state = False  # To get the initial scan data at the beginning

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
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 obstacle detection node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.init_scan_state = True

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True:
            self.detect_obstacle()

    def detect_obstacle(self):
        twist = Twist()
        obstacle_distance = min(self.scan_ranges)
        safety_distance = 0.3


        if obstacle_distance > safety_distance:
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0
        elif obstacle_distance < safety_distance:
            twist.linear.x = 0.0
            twist.angular.z = self.angular_velocity
            self.get_logger().info("Obstaculos detectados cerca. Robot esquivando.")
            if obstacle_distance < safety_distance:
                twist.linear.x = self.linear_velocity/10
                twist.angular.z = -self.angular_velocity
                self.get_logger().info("Obstaculos detectados cerca. Robot esquivando.")
            elif obstacle_distance < safety_distance:
                twist.linear.x = self.linear_velocity/10
                twist.angular.z = self.angular_velocity
                self.get_logger().info("Obstaculos detectados cerca. Robot esquivando.")     
            
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
