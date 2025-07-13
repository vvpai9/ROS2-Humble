import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.subscriber = self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.vehicle = None
        self.net = None
        self.h_x, self.h_y = 400, 240  # Center of the frame
        self.groundspeed = 0.25
        self.CAMERA_RESOLUTION = (640, 480)


    def image_callback(self, msg: Image):
        image_data = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        return image_data

def get_image(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    get_image()
