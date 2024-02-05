#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image as Image_msg
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from control_node.msg import MovingInPolar
from PIL import Image
from pyzbar.pyzbar import decode

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node')
        self.pub = rospy.Publisher('error_range', MovingInPolar, queue_size=100)
        self.sub = rospy.Subscriber('camera_on', Empty, self.delay)
        self.psi1_list = []
        self.psi2_list = []
        self.movement_list = []
        self.bridge = CvBridge() # ROS 이미지 메시지와 OpenCV 이미지 객체 간 변환을 위한 객체 생성
        
    def img_callback(self, data):
        global cv_image # 다른 def에서 사용하기 위한 전역 변수 설정
        cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8") # ROS 이미지 메시지를 OpenCV 이미지 객체로 변환

    def stabilize_image(self, frame1, frame2):
        rospy.loginfo("Stabilizing the image...")
        try:
            # Convert frames to grayscale
            gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

            # Detect ORB keypoints and descriptors
            orb = cv2.ORB_create()

            # Detect keypoints and compute descriptors for the first frame
            kp1, des1 = orb.detectAndCompute(gray1, None)

            # Detect keypoints and compute descriptors for the second frame
            kp2, des2 = orb.detectAndCompute(gray2, None)

            # Ensure descriptors are not empty
            if des1 is None or des2 is None:
                return frame2

            # Ensure descriptors have the same type
            if des1.dtype != des2.dtype:
                return frame2

            # Ensure descriptors have the same dimensions
            if des1.shape[1] != des2.shape[1]:
                return frame2

            # Use BFMatcher to find the best matches between descriptors
            bf = cv2.BFMatcher()
            matches = bf.knnMatch(des1, des2, k=2)

            # Check if matches is not empty before entering the loop
            if matches:
                # Apply ratio test to obtain good matches
                good_matches = []
                for m, n in matches:
                    if m.distance < 0.75 * n.distance:
                        good_matches.append(m)

                # Estimate the homography transformation
                if len(good_matches) > 10:
                    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                    # Find homography using RANSAC
                    M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

                    # Apply homography to stabilize the second frame
                    stabilized_frame = cv2.warpPerspective(frame2, M, (frame2.shape[1], frame2.shape[0]))
                    return stabilized_frame

            # If matches is empty or not enough good matches, return the original frame2
            return frame2

        except Exception as e:
            # Handle the exception (rospy.loginfo, log, etc.)
            rospy.loginfo(f"Error: {e}")
            raise

    def draw_center_line(self, frame, qr_center, qr_box):
        rospy.loginfo("Drawing the center line...")
        pi = np.pi

        # Get the center of the frame
        frame_center = np.array([frame.shape[1] / 2, frame.shape[0] / 2])

        # Get the direction vector from the center of the frame to the center of the QR code
        direction_vector = frame_center - qr_center

        # Calculate the angle (theta) of the line
        theta = np.arctan2(direction_vector[1], -direction_vector[0]) * 180 / pi

        # Get the orientation of the specific line of the bounding box
        box_orientation = np.arctan2(qr_box[2][0][1] - qr_box[1][0][1], qr_box[2][0][0] - qr_box[1][0][0])

        if box_orientation >= -pi/4 :
            box_orientation = box_orientation
            box_orientation_deg = box_orientation *180/pi

        elif -3/4*pi <= box_orientation < -pi/4 :
            box_orientation = box_orientation + pi/2
            box_orientation_deg = box_orientation *180/pi

        else:
            box_orientation = box_orientation + pi
            box_orientation_deg = box_orientation *180/pi

        # Calculate x, y in meter
        x_coordinate = direction_vector[0] * -0.0002375
        y_coordinate = direction_vector[1] * 0.0002375

        # rospy.loginfo the vector form (x, y, theta) in meter and radian
        # rospy.loginfo(f"Direction Vector: ({x_coordinate}, {y_coordinate}, {theta}), Box Angle: {box_orientation_deg}")

        # Draw a line from the center of the frame to the center of the QR code
        cv2.line(frame, tuple(map(int, frame_center)), tuple(map(int, qr_center)), (255, 0, 0), 2)

        # Define the new coordinate system (x', y') based on the bounding box orientation
        x_prime = np.array([np.cos(box_orientation), np.sin(box_orientation)])
        y_prime = np.array([-np.sin(box_orientation), np.cos(box_orientation)])

        # Normalize y_prime to make its magnitude equal to 1
        x_prime *= 50
        y_prime = y_prime / np.linalg.norm(y_prime)

        # Scale the normalized y_prime vector to the desired magnitude (15cm in this case)
        desired_magnitude = 0.18/0.0002375
        y_prime *= desired_magnitude

        # Draw the new coordinate axes on the image with the origin at the center of the QR code
        endpoint_x = qr_center + x_prime
        endpoint_y = qr_center + y_prime

        cv2.arrowedLine(frame, tuple(map(int, qr_center)), tuple(map(int, endpoint_x)), (0, 255, 0), 2)
        cv2.arrowedLine(frame, tuple(map(int, qr_center)), tuple(map(int, endpoint_y)), (0, 0, 255), 2)

        # Add three vectors: y_prime, direction_vector, and (0, 631)
        result_vector = y_prime - direction_vector + np.array([0, -757.894736])

        # Calculate the magnitude and angle of the resulting vector
        result_magnitude = np.linalg.norm(result_vector) * 0.0002375
        result_angle = np.arctan2(result_vector[1], result_vector[0]) * 180 / pi

        # rospy.loginfo the magnitude and angle of the resulting vector
        # rospy.loginfo(f"Resulting Vector Magnitude: {result_magnitude}, Resulting Vector Angle: {result_angle} degrees")

        # Calculate psi1 and psi2
        psi1 = pi / 2 + np.radians(result_angle)
        psi2 = - pi / 2 + box_orientation - np.radians(result_angle)
        movement = result_magnitude

        if abs(psi1) > pi :
            if psi1 < 0 :
                psi1 += 2 * pi
            else:
                psi1 -= 2 * pi

        if abs(psi2) > pi :
            if psi2 < 0 :
                psi2 += 2 * pi
            else:
                psi2 -= 2 * pi

        # rospy.loginfo the values of psi1, movement and psi2 in degree and meter
        # rospy.loginfo(f"psi1: {np.degrees(psi1)} degrees, movement: {movement} psi2: {np.degrees(psi2)} degrees")

        # Return the calculated values
        return float(psi1), float(psi2), float(movement)
    
    def init_filter_list(self):
        self.psi1_list = []
        self.psi2_list = []
        self.movement_list = []
                
    def average_filter(self, psi1, psi2, movement):
        rospy.loginfo("Averaging the values...")
        # Create a list to store the values
        psi1_list = self.psi1_list
        psi2_list = self.psi2_list
        movement_list = self.movement_list

        # Append the values to the list
        psi1_list.append(psi1)
        psi2_list.append(psi2)
        movement_list.append(movement)

        # Calculate the average of the values
        psi1_avg = sum(psi1_list) / len(psi1_list)
        psi2_avg = sum(psi2_list) / len(psi2_list)
        movement_avg = sum(movement_list) / len(movement_list)
        len = len(psi1_list)

        # rospy.loginfo the average values
        # rospy.loginfo(f"psi1_avg: {psi1_avg}, psi2_avg: {psi2_avg}, movement_avg: {movement_avg}")

        # Return the average values
        return psi1_avg, psi2_avg, movement_avg, len
    
    def pub_error_range(self, psi1, psi2, movement):
        rospy.loginfo("Publishing the error range...")
        # Check if the values are within the specified error range
        error_distance = 0.05
        error_angle = 0.157
        if abs(psi1) <= error_angle and abs(psi2) <= error_angle and abs(movement) <= error_distance:
            status = True
        else:
            status = False

        # Create and publish the MovingInPolar message
        moving_msg = MovingInPolar()
        moving_msg.status = status
        moving_msg.psi1 = psi1
        moving_msg.psi2 = psi2
        moving_msg.movement = movement
        self.pub.publish(moving_msg)
        rospy.loginfo(f'Published: {moving_msg}')

    def delay(self, msg):
        rospy.loginfo(f"Delaying for {2} seconds...")
        rospy.sleep(2)
        self.camera_on_callback(msg)
               
    def camera_on_callback(self, msg):
        # Set the desired resolution and fps 1280 x 960 doesn't work since it is not supported in camera v2
        desired_width = 640
        desired_height = 480 
        desired_fps = 15 
        
        psi1 = 0.0
        psi2 = 0.0
        movement = 0.0
        
        breaking = False
        
        cap = cv2.VideoCapture(0)

        # Set the resolution
        cap.set(3, desired_width)  # Set the width
        cap.set(4, desired_height)  # Set the height

        # Set the frames per second
        cap.set(cv2.CAP_PROP_FPS, desired_fps)

        while True:
            try:

                # Capture the first frame
                _, frame1 = cap.read()

                if breaking == True:
                    break
                
                while True:
                    # Capture the second frame
                    _, frame2 = cap.read()

                    # Stabilize the frames
                    stabilized_frame = self.stabilize_image(frame1, frame2)

                    # Perform QR code decoding on the stabilized frame
                    decoded_objects = decode(Image.fromarray(stabilized_frame))

                    # Draw a rectangle and a line around the QR code if detected
                    if decoded_objects:
                        for obj in decoded_objects:
                            qr_data = obj.data.decode("utf-8")
                            rospy.loginfo(f'Type: {obj.type}, Data: {qr_data}')

                            # Get the bounding box points of the QR code
                            points = obj.polygon
                            if len(points) == 4:
                                pts = np.array(points, dtype=np.int32)
                                pts = pts.reshape((-1, 1, 2))
                                cv2.polylines(stabilized_frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                                # Draw a line from the center of the frame to the center of the QR code
                                qr_center = np.mean(pts, axis=0).astype(int).reshape(-1)
                                psi1, psi2, movement = self.draw_center_line(stabilized_frame, qr_center, pts)
                                self.average_filter(psi1, psi2, movement)
                                
                                if self.average_filter.len == 10:
                                    breaking = True
                                    self.init_filter_list()
                                
                                # # Save the stabilized frame with the rectangle and line as an image
                                # cv2.imwrite('stabilized_frame_with_rectangle_and_line.jpg', stabilized_frame)
                                
                    if breaking == True:
                        break
                    # Update frame1 for the next iteration
                    frame1 = frame2


            except KeyboardInterrupt:
                rospy.loginfo("Program interrupted by user.")
                break
            except Exception as e:
                rospy.loginfo(f"Error: {e}")
                raise
        self.pub_error_range(psi1, psi2, movement)

if __name__ == "__main__":
    try:
        camera_node = CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt exception.")
        raise