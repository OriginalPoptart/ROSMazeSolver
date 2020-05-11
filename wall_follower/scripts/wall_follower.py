#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from move_robot import MoveRosBots
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


from tf.transformations import euler_from_quaternion
import math


class Controller(object):

    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)             
        self.sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.callback)
        self.moverosbots_object = MoveRosBots()

        self.E = 0.
        self.old_error = 0.
        self.counter = 0
    
    def trajectory_fuction(self, x, y):
        return x_d, y_d

    def callback(self, msg):
        left = msg.ranges[541:]
        front_left = msg.ranges[360:540]
        front_right = msg.ranges[180:359]
        right = msg.ranges[:179]

        var = Twist()

        # Left Turn
        if left[90] > 1:
            print "Turning Left..."
            var.linear.x = .2
            var.angular.z = .5
        # Right Turn
        elif (front_left[90] < 1):
            print "Turning right..."
            var.linear.x = .2
            var.angular.z = -.5
        # Straight Path
        else:
            print "Going Straight"
            var.linear.x = .25
            var.angular.z = 0
            wall_distance = left[90]
            #r_wall_distance = right[0]
            print wall_distance
            if wall_distance < 5:
                var.angular.z -= .5 * (.5 - wall_distance) 
                #var.angular.z += .5 * (.65 - r_wall_distance)


        self.pub.publish(var)

    def clean_up(self):
        self.moverosbots_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    controller = Controller()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        controller.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()