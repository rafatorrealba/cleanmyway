#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist #
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

qos = QoSProfile(depth=10)

def main(args=None):
    rclpy.init(args=args)
e
    def callback(dt):
        # imprime la info de los rangos del sensor en 3 angulos distintos, donde el 0 representa adelante, 15 izquierda y 345 derecha.
        print ('-------------------------------------------')
        print ('Range data at 0 deg:   {}'.format(dt.ranges[0]))
        print ('Range data at 15 deg:  {}'.format(dt.ranges[15]))
        print ('Range data at 345 deg: {}'.format(dt.ranges[345]))
        print ('-------------------------------------------')
        distancia1 = 0.8 # distancia con respecto al objeto
        distancia2 = 0.8

        if dt.ranges[0]>distancia1 and dt.ranges[15]>distancia2 and dt.ranges[345]>distancia2: # Checa si hay obstaculos en frente y a 15 grados de izquierda a derecha
            move.linear.x = 0.5 # ve en linea recta (linear velocity)
            move.angular.z = 0.0 # no gira (angular velocity)

        else:
                move.linear.x = 0.0 # se detiene
                move.angular.z = 0.5 # rota en sentido anti horario
                if dt.ranges[0]>distancia1 and dt.ranges[15]>distancia2 and dt.ranges[345]>distancia2:
                    move.linear.x = 0.5
                    move.angular.z = 0.0



        pub.publish(move)


    node = rclpy.create_node('detectar_obstaculos')
    move = Twist() # crea mensajes
    pub = node.create_publisher(Twist, "/cmd_vel", qos)  # Publisher
    sub = node.create_subscription(LaserScan, "/scan", callback, qos_profile=qos_profile_sensor_data)  # Subscriber
    rclpy.spin(node)



if __name__ == '__main__':
    main()
