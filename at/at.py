import time
import math
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import argparse
from datetime import datetime

# Parameters
ALTITUDE = 15  # Desired altitude in meters
GROUND_SPEED = 8  # Ground speed in m/s
GRID_SIZE = 20  # Size of each grid cell in meters (10m x 10m)
DETECTED_OBJECTS = 0  # Counter for detected objects

class DroneMission(Node):
    def __init__(self):
        super().__init__("drone_mission")
        self.subscriber = self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.vehicle = None
        self.net = None
        self.classNames = []
        self.h_x, self.h_y = 400, 240  # Center of the frame
        self.groundspeed = 0.25
        self.CAMERA_RESOLUTION = (1280, 720)
        self.connect_vehicle()
        self.load_model()
        self.home_lat = self.vehicle.location.global_frame.lat
        self.home_lon = self.vehicle.location.global_frame.lon
        self.geofence = [
            (self.home_lat, self.home_lon),  # Bottom-Left
            (self.home_lat, 149.16631139),  # Bottom-Right
            (-35.36190824, 149.16631139),  # Top-Right
            (-35.36190824, self.home_lon)   # Top-Left
        ]

    def connect_vehicle(self):
        # Parse connection argument
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='127.0.0.1:14550')
        args = parser.parse_args()

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % args.connect)
        self.vehicle = connect(args.connect, baud=57600, wait_ready=True, timeout=60)

    def load_model(self):
        # Load class names from the file
        classFile = "/home/pai/coco.names"
        with open(classFile, "rt") as f:
            self.classNames = f.read().rstrip("\n").split("\n")

        # Paths to configuration and weights files
        configPath = "/home/pai/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
        weightsPath = "/home/pai/frozen_inference_graph.pb"

        # Initialize the neural network model
        self.net = cv2.dnn_DetectionModel(weightsPath, configPath)
        self.net.setInputSize(320, 320)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)

    def image_callback(self, msg: Image):
        image_data = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        self.detect(image_data)

    def detect(self, image_data):
        def getObjects(img, thres, nms, draw=True, objects=[]):
            classIds, confs, bbox = self.net.detect(img, confThreshold=thres, nmsThreshold=nms)
            if len(objects) == 0:
                objects = self.classNames
            objectInfo = []
            if len(classIds) != 0:
                for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    className = self.classNames[classId - 1]
                    if className in objects:
                        objectInfo.append([box, className, confidence])
                        if draw:
                            cv2.rectangle(img, box, color=(0, 255, 0), thickness=2)
                            cv2.putText(img, className.upper(), (box[0] + 10, box[1] + 30),
                                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
                            cv2.putText(img, str(round(confidence * 100, 2)), (box[0] + 200, box[1] + 30),
                                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
            return img, objectInfo

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

        def save_image_with_info(img):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            location = self.vehicle.location.global_frame
            filename = f"image_{timestamp}_lat{location.lat}_lon{location.lon}_alt{location.alt}.jpg"
            cv2.imwrite(filename, img)
            print(f"Saved image: {filename}")

        def distance_from_center(box):
            box_center = (box[0] + box[2] // 2, box[1] + box[3] // 2)
            dist_x = box_center[0] - self.h_x
            dist_y = box_center[1] - self.h_y
            return math.sqrt(dist_x ** 2 + dist_y ** 2), dist_x, dist_y

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

    def arm_and_takeoff(self, altitude):
        print("Performing pre-arm checks...")
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(altitude)

        while True:
            print("Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def get_distance_metres(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def goto_location(self, to_lat, to_lon, alt):
        target_location = LocationGlobalRelative(to_lat, to_lon, alt)
        self.vehicle.simple_goto(target_location, groundspeed=GROUND_SPEED)

        while self.vehicle.mode.name == "GUIDED":
            current_location = self.vehicle.location.global_relative_frame
            distance = self.get_distance_metres(current_location, target_location)
            print(f"Distance to target: {distance} meters")
            if distance <= 1.5:
                print("Reached target location")
                break
            time.sleep(1)

    def generate_grid(self, geofence, grid_size):
        bottom_left, bottom_right, top_right, top_left = geofence
        lat_start, lon_start = bottom_left
        lat_end, lon_end = top_right

        # Calculate the number of grid cells needed
        lat_steps = int(self.get_distance_metres(LocationGlobalRelative(lat_start, lon_start), LocationGlobalRelative(lat_end, lon_start)) / grid_size)
        lon_steps = int(self.get_distance_metres(LocationGlobalRelative(lat_start, lon_start), LocationGlobalRelative(lat_start, lon_end)) / grid_size)

        grid = []
        for i in range(lat_steps + 1):
            row = []
            for j in range(lon_steps + 1):
                lat = lat_start + (lat_end - lat_start) * (i / lat_steps)
                lon = lon_start + (lon_end - lon_start) * (j / lon_steps)
                row.append((lat, lon))
            grid.append(row)
        return grid

    def serpentine_path(self, grid):
        for i, row in enumerate(grid):
            if i % 2 == 0:
                # Move from left to right
                for point in row:
                    lat, lon = point
                    self.goto_location(lat, lon, ALTITUDE)
                    # Capture and process image data here
                    image_data = self.capture_image()
                    self.detect(image_data)
            else:
                # Move from right to left
                for point in reversed(row):
                    lat, lon = point
                    self.goto_location(lat, lon, ALTITUDE)
                    # Capture and process image data here
                    image_data = self.capture_image()
                    self.detect(image_data)

    def capture_image(self):
        # Placeholder for image capture logic
        # In practice, you would capture an image from the drone's camera here
        return np.zeros((720, 1280, 3), dtype=np.uint8)  # Placeholder: black image

    def return_to_launch(self):
        print("Returning to launch")
        self.vehicle.mode = VehicleMode("RTL")

    def execute_mission(self):
        self.arm_and_takeoff(ALTITUDE)

        grid = self.generate_grid(self.geofence, GRID_SIZE)
        self.serpentine_path(grid)

        self.return_to_launch()

        print("Mission complete")
        print(f"Total objects detected: {DETECTED_OBJECTS}")
        self.vehicle.close()

def main(args=None):
    rclpy.init(args=args)
    drone_mission = DroneMission()
    drone_mission.execute_mission()
    rclpy.spin(drone_mission)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
