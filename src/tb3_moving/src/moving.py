#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan, pi#, atan2

"""
   TurtleBot3(buger) MAX SPEED
-----------------------------------
MAX Linear  Speed: 0.22(meter /sec)
MAX Angular Speed: 2.82(degree/sec)
"""
MAX_LIN_X = 0.22
MAX_ANG_Z = 2.82

class TurtleBot3:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        self.lin_x = MAX_LIN_X / 2
        self.ang_z = MAX_ANG_Z / 2
        
        self.fstDir = 1
        self.sndDir = 1;
        self.distDir = 1;
    
    def move2goal(self):
    
        fstAngle = float(input("Set first angle: "))
        dist = float(input("Set your distance: "))
        sndAngle = float(input("Set second angle: "))
        fstAngle = -fstAngle * pi / 180
        sndAngle = -sndAngle * pi / 180; 

        if(fstAngle < 0):
            self.fstDir = -1;
        if(sndAngle < 0):
            self.sndDir = -1;  
        if(dist <0):
            self.distDir = -1;
        
        timeFstAngle = fstAngle / self.ang_z * self.fstDir
        timeDist = dist / self.lin_x * self.distDir
        timeSndAngle = sndAngle / self.ang_z * self.sndDir

        twist = Twist()
        twist.angular.z = self.ang_z * self.fstDir
        time2end = rospy.Time.now() + rospy.Duration(timeFstAngle)
        
        while(rospy.Time.now() < time2end):
            self.pub.publish(twist)
            self.rate.sleep()
    
        twist.angular.z = 0
        self.pub.publish(twist)
        
        twist.linear.x = self.lin_x * self.distDir
        time2end = rospy.Time.now() + rospy.Duration(timeDist)
        while(rospy.Time.now() < time2end): 
            self.pub.publish(twist)
            self.rate.sleep()
    
        twist.linear.x = 0
        self.pub.publish(twist)
        
        twist.angular.z = self.ang_z * self.sndDir
        time2end = rospy.Time.now() + rospy.Duration(timeSndAngle)
        while(rospy.Time.now() < time2end): 
            self.pub.publish(twist)
            self.rate.sleep()
    
        twist.angular.z = 0
        self.pub.publish(twist)


if __name__ == '__main__':
    try:
        x = TurtleBot3()
        while (not rospy.is_shutdown()):
            x.move2goal()
    except rospy.ROSInterruptException:   pass