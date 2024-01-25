#!/usr/bin/env python

import rospy
from control_node.msg import CoordInfo, MovingInPolar
from control_node.srv import ControlService, ControlServiceResponse
from std_msgs.msg import Empty

class Button():
    def __init__(self):
        rospy.Subscriber('button_info_topic', CoordInfo, self.handle_button_info)
        self.coord_info = CoordInfo()
        self.get_coord_info = rospy.ServiceProxy('get_coord_info_service', ControlService)
        self.camera = Camera()  # Camera 클래스의 인스턴스 생성

    def handle_button_info(self, msg):
        # Process the received button information here
        rospy.loginfo(f"Received button information: Label={msg.label}, X={msg.x}, Y={msg.y}, Yaw={msg.yaw}")
        self.coord_info = msg
        self.send_coord_info()
    
    def send_coord_info(self):
        # 서비스 호출 및 응답 확인
        response = self.get_coord_info(self.coord_info.x, self.coord_info.y, self.coord_info.yaw)
        if response.result:
            rospy.loginfo(f"Arrival at the {self.coord_info.label}")
            # 서비스 응답이 있을 때 camera 클래스의 turn_on 메서드 호출
            self.camera.turn_on()

class Camera():
    def __init__(self):
        rospy.loginfo("Initialize the camera")
        self.camera_on = rospy.Publisher('camera_on', Empty, queue_size=10)
        self.error = rospy.Subscriber('error', MovingInPolar, self.handle_error)

    def turn_on(self):
        rospy.loginfo("Turn on the camera")
        self.camera_on.publish()

    def handle_error(self, msg):
        if msg.status:
            rospy.loginfo("Complete the task.")

def control_node():
    rospy.init_node('control_node')
    button = Button()
    rospy.spin()

if __name__ == '__main__':
    control_node()
