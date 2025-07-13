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

class Subscriber(Node):
    def __init__(self):
        super().__init__("subscriber")
        self.subscriber = self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.vehicle = None
        self.net = None
        self.classNames = []
        self.h_x, self.h_y = 400, 240  # Center of the frame
        self.groundspeed = 0.25
        self.CAMERA_RESOLUTION = (1280, 720)
        self.connect_vehicle()

    def connect_vehicle(self):
        # Parse connection argument
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='127.0.0.1:14550')
        args = parser.parse_args()

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % args.connect)
        self.vehicle = connect(args.connect, baud=57600, wait_ready=True, timeout=60)


    def image_callback(self, msg: Image):
        image_data = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))


        def send_ned_velocity(velocity_x, velocity_y, velocity_z):
            msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,  # time_boot_ms (not used)
                0, 0,  # target system, target component
                mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
                0b10111000111,  # type_mask (only speeds enabled)
                0, 0, 0,  # x, y, z positions (not used)
                velocity_y, velocity_x, velocity_z,  # x, y, z velocity in m/s
                0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

            self.vehicle.send_mavlink(msg)
            self.vehicle.flush()


        def landavi(img):
            print("Landing")

            initial_position = self.vehicle.location.global_relative_frame

            while True:
                result, objectInfo = getObjects(img, 0.45, 0.2, objects=['person'])

                if objectInfo:
                    # Sort objects by distance from the center
                    objectInfo.sort(key=lambda obj: distance_from_center(obj[0])[0])

                    # Process each detected person
                    for idx, obj in enumerate(objectInfo, start=1):
                        box = obj[0]
                        distance, dist_x, dist_y = distance_from_center(box)

                        # Label the person with their order number
                        cv2.putText(img, f'Person {idx}', (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

                        while distance > 10 and self.vehicle.location.global_relative_frame.alt >= 1:
                            velocity_x = max(min(self.groundspeed * dist_x / distance, self.groundspeed), -self.groundspeed)
                            velocity_y = max(min(self.groundspeed * dist_y / distance, self.groundspeed), -self.groundspeed)
                            velocity_z = self.groundspeed

                            send_ned_velocity(velocity_x, velocity_y, velocity_z)

                            time.sleep(0.1)
                            distance, dist_x, dist_y = distance_from_center(box)

                        if distance <= 10:
                            save_image_with_info(img)

                        # Return to the original position
                        self.vehicle.simple_goto(initial_position)
                        while self.vehicle.mode.name == "GUIDED" and self.vehicle.location.global_relative_frame.alt > 1:
                            print("Returning to initial position")
                            time.sleep(1)
                        print("Reached initial position")

                cv2.imshow("Output", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        landavi(image_data)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

