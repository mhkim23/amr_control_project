#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pow, sqrt, atan, pi#, atan2
from control_node.msg import MovingInPolar
from std_msgs.msg import Empty


"""
   TurtleBot3(buger) MAX SPEED
-----------------------------------
MAX Linear  Speed: 0.22(meter /sec)
MAX Angular Speed: 2.82(radians/sec)
"""

MAX_LIN_X = 0.22
MAX_ANG_Z = 0.82

class MoveBot:

    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('error_range', MovingInPolar, self.callbackfunc)
        self.pubb = rospy.Publisher('camera_on', Empty, queue_size=10)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.pose = MovingInPolar()
        rospy.loginfo("TurtleBot3 Moving To Goal")
        
        self.lin_x = MAX_LIN_X / 2
        self.ang_z = MAX_ANG_Z / 2
        
        self.fstDir = 1
        self.sndDir = 1
        self.distDir = 1
    
        self.fstAngle = self.pose.psi1
        self.dist = self.pose.movement
        self.sndAngle = self.pose.psi2

        #CW : Positive, CCW : Negative
        self.fstAngle = -self.fstAngle * pi / 180
        self.sndAngle = -self.sndAngle * pi / 180
    
        if(self.fstAngle < 0):
            self.fstDir = -1
        if(self.sndAngle < 0):
            self.sndDir = -1  
        if(self.dist <0):
            self.distDir = -1
    
    def callbackfunc(self, msg):
        self.pose = msg
        rospy.loginfo("initailize callbakcfunc")
        if self.pose.status == False:
            rospy.loginfo("status is false")
            self.move2goal()
        
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
        return self.pubb.publish()

if __name__ == '__main__':
    try:
        move = MoveBot()
        rospy.spin()
    except rospy.ROSInterruptException:   pass