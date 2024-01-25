#!/usr/bin/env python

import rospy
from control_node.srv import ControlService, ControlServiceResponse

def handle_get_coord_info(req):
    rospy.loginfo(f"Received button information: X={req.x}, Y={req.y}, Yaw={req.yaw}")
    result = "Received button information"
    return ControlServiceResponse(result)

def receive_coord_info_server():
    rospy.init_node('receive_coord_info_server')
    service = rospy.Service('get_coord_info_service', ControlService, handle_get_coord_info)
    rospy.spin()

if __name__ == '__main__':
    receive_coord_info_server()
