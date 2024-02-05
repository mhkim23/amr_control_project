#!/usr/bin/env python

import rospy
from control_node.msg import CoordInfo  # Import your custom message type for topic
import tkinter as tk

class MyGUI:
    def __init__(self, master, publisher):
        self.master = master
        self.publisher = publisher
        self.create_buttons()
        self.create_quit_button()

    def create_buttons(self):
        button_info = [
            {"label": "CNC1", "x": 1.3779, "y": -1.3094, "yaw": 2.96706},
            {"label": "CNC2", "x": 1.5051, "y": -0.4361, "yaw": 2.96706},
            {"label": "CNC3", "x": 1.6964, "y": 0.4525, "yaw": 2.96706},
            {"label": "CNC4", "x": 2.2283, "y": -1.4577, "yaw": 2.96706},
            {"label": "CNC5", "x": 2.3801, "y": -0.5451, "yaw": 2.96706},
            {"label": "CNC6", "x": 2.521, "y": 0.387, "yaw": 2.96706},
            {"label": "Initialize", "x": 1.3591, "y": -1.3385, "yaw": 2.96706}
        ]

        for i, info in enumerate(button_info):
            button = tk.Button(self.master, text=info["label"], command=lambda i=info: self.send_button_info(i))
            row = i // 2
            col = i % 2
            button.grid(row=row, column=col, padx=10, pady=10)

    def create_quit_button(self):
        quit_button_info = {"label": "Quit", "x": -1.1, "y": 0.91, "yaw": 1.570796}
        quit_button = tk.Button(self.master, text=quit_button_info["label"], command=self.master.destroy)
        quit_button.grid(row=3, column=1)

    def send_button_info(self, info):
        message = CoordInfo()
        message.label = info['label']
        message.x = info['x']
        message.y = info['y']
        message.yaw = info['yaw']
        self.publisher.publish(message)
        rospy.loginfo(f"{info['label']} 버튼 클릭 메시지를 발행했습니다.")

def button_node():
    rospy.init_node('button_node')

    publisher = rospy.Publisher('button_info_topic', CoordInfo, queue_size=10)

    root = tk.Tk()
    root.title("버튼 노드 GUI")

    gui = MyGUI(root, publisher)

    root.mainloop()

if __name__ == '__main__':
    button_node()
