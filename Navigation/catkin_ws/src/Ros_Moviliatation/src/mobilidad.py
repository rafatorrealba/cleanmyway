#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

turn_velocity = 7
linear_velocity = 0.5


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

rospy.init_node('sub_node')
sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist, queue_size= 4 )
move = Twist()

rospy.sleep(1000000)