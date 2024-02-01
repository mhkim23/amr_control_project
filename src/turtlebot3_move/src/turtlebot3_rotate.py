#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from control_node.msg import CoordInfo
from tf.transformations import quaternion_from_euler
    
def Rotate():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()
    twist.angular.z = 3.14/4
    time2end = rospy.Time.now() + rospy.Duration(16)
    
    #Rotate
    while(rospy.Time.now() < time2end):
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    pub.publish(twist)

def Initialize() :
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = rospy.Rate(10)
    start = PoseWithCovarianceStamped()
    start.header.frame_id = 'map'
    start.header.stamp = rospy.Time.now()
    start.pose.pose.position.x = 1.37
    start.pose.pose.position.y = -1.343
    start.pose.pose.position.z = 0.0
    q = quaternion_from_euler(0,0,1.57)
    start.pose.pose.orientation.x = q[0];
    start.pose.pose.orientation.y = q[1];
    start.pose.pose.orientation.z = q[2];
    start.pose.pose.orientation.w = q[3];
    start.pose.covariance[0] = 0.25
    start.pose.covariance[7] = 0.25
    start.pose.covariance[35] = 0.06853892326654787
    count = 0
    while(count < 10):
        start.header.stamp = rospy.Time.now();
        pub.publish(start)
        count+=1
        rate.sleep()

def callbackfunc(msg):
    if(msg.label == 'Initialize'):
        Initialize()
        Rotate()

if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('button_info_topic', CoordInfo, callbackfunc)
        rospy.spin()
    except rospy.ROSInterruptException:   pass