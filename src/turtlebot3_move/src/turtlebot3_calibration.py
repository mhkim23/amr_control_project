#!/usr/bin/env python3

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
        
        self.lin_x = MAX_LIN_X / 4
        self.ang_z = MAX_ANG_Z / 4
        
        self.fstDir = 1
        self.sndDir = 1
        self.distDir = 1
        
        self.fstAngle = 0
        self.dist = 0
        self.sndAngle = 0

    
    def setParam(self, fstAngle, dist, sndAngle):
        self.fstAngle = fstAngle
        self.dist = dist
        self.sndAngle = sndAngle
        
        #CW : Positive, CCW : Negative
        self.fstAngle = -fstAngle 
        self.sndAngle = -sndAngle
        self.dist = dist
        # rospy.loginfo(f"fstAngle: {self.fstAngle}, dist: {self.dist}, sndAngle: {self.sndAngle}")
        if(self.fstAngle < 0):
            self.fstDir = -1
        if(self.sndAngle < 0):
            self.sndDir = -1  

    
    def callbackfunc(self, msg):
        self.pose = msg
        self.setParam(self.pose.psi1, self.pose.movement, self.pose.psi2)
        rospy.loginfo(f"msg: {msg}")
        rospy.loginfo("initailize callbackfunc")
        if self.pose.status == False:
            rospy.loginfo("status is false")
            self.move2goal()
        
    def move2goal(self):
        rospy.loginfo("move2goal")
        timeFstAngle = abs(self.fstAngle / self.ang_z)
        timeDist = abs(self.dist / self.lin_x)
        timeSndAngle = abs(self.sndAngle / self.ang_z)
        rospy.loginfo(f"timeFstAngle: {timeFstAngle}, timeDist: {timeDist}, timeSndAngle: {timeSndAngle}")

        twist = Twist()

        #rotate psi1
        twist.angular.z = self.ang_z * self.fstDir
        time2end = rospy.Time.now() + rospy.Duration(timeFstAngle)
        while(rospy.Time.now() < time2end):
            # rospy.loginfo(f"rotate psi1 {twist}")
            self.pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        self.pub.publish(twist)

        #move dist
        twist.linear.x = self.lin_x * self.distDir
        time2end = rospy.Time.now() + rospy.Duration(timeDist)
        while(rospy.Time.now() < time2end): 
            self.pub.publish(twist)
            # rospy.loginfo(f"move dist {twist}")
            self.rate.sleep()
    
        twist.linear.x = 0
        self.pub.publish(twist)

        #rotate psi2
        twist.angular.z = self.ang_z * self.sndDir
        time2end = rospy.Time.now() + rospy.Duration(timeSndAngle)
        while(rospy.Time.now() < time2end): 
            self.pub.publish(twist)
            rospy.loginfo(f"rotate psi2 {twist}")
            self.rate.sleep()
        twist.angular.z = 0
        self.pub.publish(twist)
        
        # return self.pubb.publish()

if __name__ == '__main__':
    try:
        move = MoveBot()
        rospy.spin()
    except rospy.ROSInterruptException:   pass