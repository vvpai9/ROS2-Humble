# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from ultralytics import YOLO  # YOLO library
import numpy as np

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
          
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            'camera', 
            self.listener_callback, 
            10)
        self.subscription  # prevent unused variable warning
          
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
   
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
     
        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # Debug: Check the type and shape of the image
            self.get_logger().info(f'Image type: {type(current_frame)}, Image shape: {current_frame.shape}')

            # Ensure the image is a NumPy array
            if not isinstance(current_frame, np.ndarray):
                raise ValueError("Image is not a valid NumPy array")

            # Object Detection
            results = model.predict(source=current_frame, cls=[0, 2])

            # Ensure results is a list and retrieve the first result
            if isinstance(results, list) and len(results) > 0:
                img = results[0].plot()
                
                # Show Results
                cv2.imshow('Detected Frame', img)
                cv2.waitKey(1)
            else:
                self.get_logger().error('No detection results to display')

        except Exception as e:
            self.get_logger().error(f'Error during prediction: {e}')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = ImageSubscriber()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()

