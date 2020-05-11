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
import time


from tf.transformations import euler_from_quaternion
import math

class Junction():

    def __init__(self, x, y, angle, direction):
        self.x = x
        self.y = y
        self.angle = angle
        self.direction = direction
        self.been_to = ''


class Controller(object):

    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)             
        self.sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.callback)
        self.sub2 = rospy.Subscriber("/odom", Odometry, self.callback2)
        self.moverosbots_object = MoveRosBots()

        self.E = 0.
        self.old_error = 0.
        self.counter = 0

        self.x = 0
        self.y = 0
        self.angle = 0
        self.flush = 0

        self.junctions = []
    

    def callback(self, msg):
        left = msg.ranges[541:]
        front_left = msg.ranges[360:540]
        front_right = msg.ranges[180:359]
        right = msg.ranges[:179]

        var = Twist()
        print "Current Coords: (%.2f, %.2f)"%(self.x, self.y)
        print "Distances LFR: (%.2f, %.2f, %.2f)"%(left[90], front_left[0], right[90])
        print "Flush = " + str(self.flush)

        distance = 3
        if self.flush > 0:
            self.flush -= 1
        elif left[90] > distance and right[90] > distance and front_left[0] > distance:
            print "Adding Junction LFR"
            junc = self.find_coords()
            if junc == None:
                junc = Junction(self.x, self.y, self.angle, 'lfr')
                self.junctions.append(junc)
            # turn a direction we haven't been yet
            if junc.been_to.find('l') == -1:
                # go forward 
                print "Going left"
                junc.been_to += 'l'
                var.linear.x = .3
                var.angular.z = -.3
                self.pub.publish(var)
                time.sleep(6)
            elif junc.been_to.find('r') == -1:
                # turn right
                print "Turning Right"
                junc.been_to += 'r'
                var.linear.x = .3
                var.angular.z = -.3
                self.pub.publish(var)
                time.sleep(6)
            elif junc.been_to.find('f') == -1:
                # go forward 
                print "Going straight"
                junc.been_to += 'f'
                var.linear.x = .5
                var.angular.z = 0
                self.pub.publish(var)
                time.sleep(2)
            
            self.flush = 100
        elif left[90] > distance and right[90] > distance:
            print "Adding Junction LR"
            junc = self.find_coords()
            if junc == None:
                junc = Junction(self.x, self.y, self.angle, 'fr')
                self.junctions.append(junc)
            # turn a direction we haven't been yet
            if junc.been_to.find('r') == -1:
                # turn right
                print "Turning Right"
                junc.been_to += 'r'
                var.linear.x = .3
                var.angular.z = -.3
                self.pub.publish(var)
                time.sleep(6)
            elif junc.been_to.find('l') == -1:
                # go forward 
                print "Going left"
                junc.been_to += 'l'
                var.linear.x = .3
                var.angular.z = .3
                self.pub.publish(var)
                time.sleep(3)
            self.flush = 100        
        elif front_left[0] > distance and right[90] > distance:
            print "Adding Junction FR"
            junc = self.find_coords()
            if junc == None:
                junc = Junction(self.x, self.y, self.angle, 'fr')
                self.junctions.append(junc)
            # turn a direction we haven't been yet
            if junc.been_to.find('r') == -1:
                # turn right
                print "Turning Right"
                junc.been_to += 'r'
                var.linear.x = .3
                var.angular.z = -.3
                self.pub.publish(var)
                time.sleep(6)
            elif junc.been_to.find('f') == -1:
                # go forward 
                print "Going straight"
                junc.been_to += 'f'
                var.linear.x = .5
                var.angular.z = 0
                self.pub.publish(var)
                time.sleep(2)
            self.flush = 100
        elif left[90] > distance and front_left[0] > distance:
            print "Adding Junction LF"
            junc = self.find_coords()
            if junc == None:
                junc = Junction(self.x, self.y, self.angle, 'fr')
                self.junctions.append(junc)
            # turn a direction we haven't been yet
            if junc.been_to.find('l') == -1:
                # turn right
                print "Turning Left"
                junc.been_to += 'l'
                var.linear.x = .3
                var.angular.z = .3
                self.pub.publish(var)
                time.sleep(6)
            elif junc.been_to.find('f') == -1:
                # go forward 
                print "Going straight"
                junc.been_to += 'f'
                var.linear.x = .5
                var.angular.z = 0
                self.pub.publish(var)
                time.sleep(2)
            self.flush = 100
        else:
            # Left Turn
            if left[90] > 1:
                #print "Turning Left..."
                var.linear.x = .2
                var.angular.z = .5
            # Right Turn
            elif (front_left[90] < 1):
                #print "Turning right..."
                var.linear.x = .2
                var.angular.z = -.5
            # Straight Path
            else:
                #print "Going Straight"
                var.linear.x = .25
                var.angular.z = 0
                wall_distance = left[90]
                #r_wall_distance = right[0]
                #print wall_distance
                if wall_distance < 5:
                    var.angular.z -= .5 * (.65 - wall_distance) 
                    #var.angular.z += .5 * (.65 - r_wall_distance)

            self.pub.publish(var)

    def callback2(self, odometry):
        self.x, self.y = odometry.pose.pose.position.x, odometry.pose.pose.position.y
        orientation_q = odometry.pose.pose.orientation
        (rot_x, rot_y, phi) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.angle = phi

    def find_coords(self):
    
        for i in self.junctions:
            if (abs(self.x - i.x) < 1 and abs(self.y - i.y) < 1 and abs(self.angle - i.angle) < 30):
                return i 

        return None

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