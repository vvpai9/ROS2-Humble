import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
import numpy as np
import cv2
import os
from datetime import datetime
from ultralytics import YOLO
from ultralytics.solutions import object_counter

class DetectionNode(Node):
    def __init__(self):
        super().__init__("detection_node")
        self.publisher = self.create_publisher(Int32MultiArray, "detection_coordinates", 10)
        self.subscription = self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.bridge = CvBridge()
        self.image_saved = False
        self.gallery_path = "gallery"
        if not os.path.exists(self.gallery_path):
            os.makedirs(self.gallery_path)
        
        # Camera intrinsic parameters
        self.focal_length = 205.13  # in pixels
        self.cx = 320  # principal point x-coordinate
        self.cy = 240  # principal point y-coordinate

        # Camera extrinsics
        self.camera_height = 20  # height of the camera above the object in meters

        # Load YOLO model
        self.model = YOLO("yolov8n.pt")
        self.counter = object_counter.ObjectCounter()
        self.region_points = [(2, 0), (0, 479), (639, 479), (639, 0)]
        self.counter.set_args(view_img=True,
                              reg_pts=self.region_points,
                              classes_names=self.model.names,
                              draw_tracks=True)

    def pixel_to_global(self, pixel_x, pixel_y):
        # Convert pixel coordinates to normalized image coordinates
        normalized_x = (pixel_x - self.cx) / self.focal_length
        normalized_y = (pixel_y - self.cy) / self.focal_length

        # Calculate object's position relative to camera
        object_z = self.camera_height
        object_x = normalized_x * object_z
        object_y = normalized_y * object_z

        # Global coordinates relative to camera
        global_x = object_x
        global_y = object_y

        return global_x, global_y

    def cam(self, image_data):
        tracks = self.model.predict(image_data)
        image_data = self.counter.start_counting(image_data, tracks)
        
        for track in tracks:
        	for box in track.boxes:
        		bbox = box.xyxy.cpu().numpy()[0].astype(int)
        		cls = box.cls.cpu().numpy()[0]
        		cv2.rectangle(image_data, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)

        cv2.imshow('Live Feed', image_data)
        cv2.waitKey(1)

    def publish_coordinates(self, x, y):
        coordinates = Int32MultiArray()
        coordinates.data = [x, y]
        self.publisher.publish(coordinates)
        self.get_logger().info(f"Published coordinates: x={x}, y={y}")

    def save_image(self, image):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = os.path.join(self.gallery_path, f"image_{timestamp}.jpg")
        cv2.imwrite(image_path, image)
        self.get_logger().info(f"Image saved: {image_path}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
        else:
            self.cam(cv_image)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

