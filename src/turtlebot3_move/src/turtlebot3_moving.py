#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import degrees, sqrt, pi, atan2

# define pi, 2pi, pi/2
_2PI = 2.0 * pi
_PI  = 1.0 * pi
_R   = 0.5 * pi

# Turtlebot3 Specification
MAX_LIN_SPEED =  0.22
MAX_ANG_SPEED =  2.84

# make default speed of linear & angular
LIN_SPD = MAX_LIN_SPEED * 0.125
ANG_SPD = MAX_ANG_SPEED * 0.125

class MoveTB3():

    def __init__(self):  
        rospy.Subscriber('/tb3pose', Pose, self.get_pose)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)   
        self.tb3pose  = self.org = Pose()
                 
    def get_pose(self, msg):
        self.tb3pose = msg
            
    def update_org(self):
        self.org = self.tb3pose
                
    def elapsed_dist(self):
        return sqrt(pow((self.tb3pose.x - self.org.x), 2) + pow((self.tb3pose.y - self.org.y), 2))
            
    def straight(self, distance): 
        tw = Twist()    
        self.update_org()
        print("start from (%s, %s)" %(round(self.org.x, 2), round(self.org.y, 2)))       
        
        if distance >= 0:   # +distance
            tw.linear.x =  LIN_SPD
        else:               # -distance
            tw.linear.x = -LIN_SPD
                        
        self.pub.publish(tw)        
        while self.elapsed_dist() < abs(distance):  pass
            #print "%s(m) of %s(m)" %(round(self.elapsed_dist(),2), round(abs(distance),2))
        
        tw.linear.x = 0;    self.pub.publish(tw)
        print("stop to (%s, %s)." %(round(self.tb3pose.x, 2), round(self.tb3pose.y, 2)))   
        
    def elapsed_angle(self):
        return abs(self.tb3pose.theta - self.org.theta)
        
    def rotate(self, angle):
        tw = Twist()
        self.update_org()
        print("start from: %s" %(round(degrees(self.org.theta), 2)))
        
        if angle >= 0:	# angle(+): rotate left(ccw)
            tw.angular.z =  ANG_SPD;
        else:			# angle(-): rotate right(cw)
            tw.angular.z = -ANG_SPD;
            
        self.pub.publish(tw)
        while self.elapsed_angle() < abs(angle):    pass
            # print "%s of %s" %(round(degrees(self.elapsed_angle()),2) ,round(degrees(abs(angle)),2))
            
        tw.angular.z =  0;  self.pub.publish(tw)
        print("stop to   : %s" %(round(degrees(self.tb3pose.theta), 2)))
        
class MoveTo_XY():

    def __init__(self):
        self.move = MoveTB3()
    
    def move2xy(self, x, y, theta):
        
        angle = atan2(y, x)
        dist  = sqrt(pow(x, 2) + pow(y, 2))
        
        print("rotate %s(deg), and than straight %s(m)" %(degrees(angle), dist))
        
        self.move.rotate(angle)
        self.move.straight(dist)
        self.move.rotate(-angle + theta)

# 좌표값 받아오는 함수
#class GetCoordinate():
# def __init__():
#     rospy.Subscriber('/cameraCoordinate', msg, callbackfunc())   
# def callbackfunc(msg):
#     x = msg.x
#     y = msg.y
#     theta = msg.theta
#     return x, y, theta
    
if __name__ == '__main__':

    try:
        rospy.init_node('move_to_xy', anonymous = True)        
        xy = MoveTo_XY()
        #coord = GetCoordinate()
        
        while not rospy.is_shutdown():
            dist_x = float(input("input distance x(m): "))
            dist_y = float(input("input distance y(m): "))
            theta = float(input("input theta(deg): "))
            # dist_x, dist_y, theta = coord.callbackfunc()
            xy.move2xy(dist_x, dist_y, theta)
            
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass
