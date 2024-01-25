#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from control_node.msg import CoordInfo

    
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

def callbackfunc(msg):
    if(msg.label == 'Initialize'):
        Rotate()

if __name__ == '__main__':
    try:
        rospy.init_node('turtlebot_controller', anonymous=True)
        rospy.Subscriber('button_info_topic', CoordInfo, callbackfunc)
        rospy.spin()
    except rospy.ROSInterruptException:   pass
