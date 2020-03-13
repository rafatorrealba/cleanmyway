#!/usr/bin/env python3

import rclpy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data


class ObstacleDetection(Node):

    def __init__(self):
        super().__init__('obstacle_detection')

        # Iniciar variables
        self.distancia1 = 0.8 # distancia con respecto al objeto
        self.distancia2 = 0.8
        self.init_scan_state = False

        # Initialise ROS publishers and subscribers
        qos = QoSProfile(depth=10)

        # Iniciar publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Iniciar subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.detect_obstacle,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        #Initialise timers
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("La deteccion de obscatulos de Turtlebot3 se ha iniciado")
        

    #Callback functions and relevant function

    def cmd_vel_raw_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_callback(self):
        if self.init_scan_state is True:
            self.detect_obstacle()

    def detect_obstacle(self, dt):
        move = Twist()
        if dt.ranges[0]>self.distancia1 and dt.ranges[15]>self.distancia2 and dt.ranges[345]>self.distancia2:
             # Checa si hay obstaculos en frente y a 15 grados de izquierda a derecha
             move.linear.x = 0.5 # ve en linea recta (linear velocity)
             move.angular.z = 0.0 # no gira (angular velocity)
        else:
                move.linear.x = 0.0 # se detiene
                move.angular.z = 0.5 # rota en sentido anti horario
                self.get_logger().info("Obstaculos cerca detectados. Robot esquivando.")
                if dt.ranges[0]>self.distancia1 and dt.ranges[15]>self.distancia2 and dt.ranges[345]>self.distancia2:
                    move.linear.x = 0.5
                    move.angular.z = 0.0


        self.cmd_vel_pub.publish(move)


def main(args=None):
	rclpy.init(args=args)
	turtlebot3_obstacle_detection = ObstacleDetection()
	rclpy.spin(turtlebot3_obstacle_detection)




if __name__ == '__main__':
    main()
