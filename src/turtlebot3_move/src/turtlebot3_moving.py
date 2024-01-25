#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan, pi#, atan2
from control_node import MovingInPolar

# 좌표값 받아오는 함수
#class GetCoordinate():
# def __init__():
#     rospy.Subscriber('/cameraCoordinate', msg, callbackfunc())   
# def callbackfunc(msg):
#     x = msg.x
#     y = msg.y
#     theta = msg.theta
#     return x, y, theta

"""
   TurtleBot3(buger) MAX SPEED
-----------------------------------
MAX Linear  Speed: 0.22(meter /sec)
MAX Angular Speed: 2.82(radians/sec)
"""

MAX_LIN_X = 0.22
MAX_ANG_Z = 2.82

class Coord():

    def __init__(self):
        rospy.Subscriber('error_range', MovingInPolar, self.callbackfunc)
        self.pose = MovingInPolar()

    def callbackfunc(self, msg):
        self.pose = msg
        if(self.pose.status == True):
            self.pose.psi1 = 0
            self.pose.psi2 = 0
            self.pose.movement = 0
    
class MoveBot:

    def __init__(self, psi1, psi2, dist):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.coord = Coord()
        
        self.lin_x = MAX_LIN_X / 2
        self.ang_z = MAX_ANG_Z / 2
        
        self.fstDir = 1
        self.sndDir = 1;
        self.distDir = 1;
    
        self.fstAngle = self.coord.psoe.psi1
        self.dist = self.coord.psoe.movenet
        self.sndAngle = self.coord.psoe.psi2

        #CW : Positive, CCW : Negative
        self.fstAngle = -self.fstAngle * pi / 180
        self.sndAngle = -self.sndAngle * pi / 180; 
    
        if(self.fstAngle < 0):
            self.fstDir = -1;
        if(self.sndAngle < 0):
            self.sndDir = -1;  
        if(self.dist <0):
            self.distDir = -1;
    
    def move2goal(self):
        
        timeFstAngle = self.fstAngle / self.ang_z * self.fstDir
        timeDist = self.dist / self.lin_x * self.distDir
        timeSndAngle = self.sndAngle / self.ang_z * self.sndDir

        twist = Twist()

        #rotate psi1
        twist.angular.z = self.ang_z * self.fstDir
        time2end = rospy.Time.now() + rospy.Duration(timeFstAngle)
        while(rospy.Time.now() < time2end):
            self.pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        self.pub.publish(twist)

        #move dist
        twist.linear.x = self.lin_x * self.distDir
        time2end = rospy.Time.now() + rospy.Duration(timeDist)
        while(rospy.Time.now() < time2end): 
            self.pub.publish(twist)
            self.rate.sleep()
    
        twist.linear.x = 0
        self.pub.publish(twist)

        #rotate psi2
        twist.angular.z = self.ang_z * self.sndDir
        time2end = rospy.Time.now() + rospy.Duration(timeSndAngle)
        while(rospy.Time.now() < time2end): 
            self.pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        self.pub.publish(twist)

if __name__ == '__main__':
    try:
        move = MoveBot()
        move.move2goal()
    except rospy.ROSInterruptException:   pass