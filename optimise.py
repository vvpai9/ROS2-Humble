import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import argparse
import time
import math
from datetime import datetime

class RedDetector(Node):
    def __init__(self):
        super().__init__("red_detector")
        self.subscriber = self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.vehicle = None
        self.net = None
        self.classNames = []
        self.h_x, self.h_y = 400, 240  # Center of the frame
        self.groundspeed = 0.25
        self.CAMERA_RESOLUTION = (1280, 720)

    def image_callback(self, msg: Image):
        # Convert ROS Image message to OpenCV image
        image_data = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))

        # Check the encoding of the image and convert it to BGR if necessary
        if msg.encoding == 'rgb8':
            image_data = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)

        self.detect(image_data)

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red color in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the HSV frame to create a mask for red colors
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # Find contours in the red mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize variables to store information about the largest contour
        largest_contour = None
        largest_contour_area = 0

        # Find the largest contour
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_contour_area:
                largest_contour = contour
                largest_contour_area = area

        # Draw a circle at the centroid of the largest contour
        if largest_contour is not None and len(largest_contour) >= 5:
            # Calculate centroid of the largest contour
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print("X coordinate:", cx)
                print("Y coordinate:", cy)
                
                with open("xa.txt", "a") as f:
                    f.write(str(cx) + "\n")
                with open("yb.txt", "a") as f:
                    f.write(str(cy) + "\n")
                
                # Draw a circle at the centroid
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        # Display the resulting frame
        cv2.imshow('Webcam', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    red_detector = RedDetector()
    rclpy.spin(red_detector)

if __name__ == '__main__':
    main()

