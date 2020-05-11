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


class Controller(object):

    def __init__(self):            
        rospy.Subscriber("/lanepoint", Point, self.callback)
        self.moverosbots_object = MoveRosBots()

        self.E = 0.
        self.old_error = 0.
        

    def callback(self, lanepoint):
        cx, cy = lanepoint.x, lanepoint.y

        # Proportional controller
        kp = 0.01
        kd = 0.1
        ki = 0.00000
        ref = 0.
        error = ref - cx   # Error

        # Control inputs
        #w = kp * error      # P Control for angular velocity
        d_error = error - self.old_error
        w = (kp * error) + (kd * d_error) + (ki * self.E)
        v = 0.4             # Linear velocity
        
        #w += -0.2
        print error

        self.E += error
        self.old_error = error

        
        #w += kd * d_error
        #w += ki * self.E

        # Message
        twist_object = Twist()
        twist_object.linear.x = v
        twist_object.angular.z = w

        # Send message
        self.moverosbots_object.move_robot(twist_object)
        
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