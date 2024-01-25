#!/usr/bin/env python

import rospy
from control_node.msg import CoordInfo
from control_node.srv import ControlService

class Button():
    def __init__(self):
        rospy.Subscriber('button_info_topic', CoordInfo, self.handle_button_info)
        self.coord_info = CoordInfo()
        self.get_coord_info = rospy.ServiceProxy('get_coord_info_service', ControlService)

    def handle_button_info(self, msg):
        # Process the received button information here
        rospy.loginfo(f"Received button information: Label={msg.label}, X={msg.x}, Y={msg.y}, Yaw={msg.yaw}")
        self.coord_info = msg
        self.send_coord_info()
    
    def send_coord_info(self):
        self.get_coord_info(self.coord_info.x, self.coord_info.y, self.coord_info.yaw)

def control_node():
    rospy.init_node('control_node')
    button = Button()
    rospy.spin()

if __name__ == '__main__':
    control_node()
