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
CAPTURE_INTERVAL = 5  # Interval between image captures in seconds

class DroneMission(Node):
    def __init__(self):
        super().__init__("drone_mission")
        self.subscriber = self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.vehicle = None
        self.previous_image = None
        self.previous_gps_coords = None
        self.image_data = None  # To store the latest image data
        self.last_capture_time = time.time()
        self.CAMERA_RESOLUTION = (1280, 720)
        self.connect_vehicle()
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

    def image_callback(self, msg: Image):
        image_data = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        self.image_data = image_data  # Update the latest image data

    def process_images_realtime(self, image):
        gps_coords = self.get_gps_data()
        accel_values = self.get_accel_data()

        if self.previous_image is not None:
            stitched_map = self.stitch_images(self.previous_image, image, self.previous_gps_coords, gps_coords, accel_values)
            cv2.imshow('Real-Time Stitched Map', stitched_map)
            cv2.waitKey(1)  # Adjust the delay as needed

        self.previous_image = image
        self.previous_gps_coords = gps_coords

    def stitch_images(self, img1, img2, gps1, gps2, accel_values):
        # Feature detection and matching
        orb = cv2.ORB_create()
        keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
        keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(descriptors1, descriptors2)
        matches = sorted(matches, key=lambda x: x.distance)

        # Ensure there are enough matches
        if len(matches) < 4:
            print("Not enough matches found. Skipping stitching.")
            return img2  # Return the second image unaltered

        src_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC)

        # Adjust homography based on GPS data
        H = self.adjust_homography_based_on_gps(H, gps1, gps2)

        height, width, channels = img1.shape
        warped_img2 = cv2.warpPerspective(img2, H, (width, height))

        stitched_img = self.blend_images(img1, warped_img2)
        return stitched_img

    def adjust_homography_based_on_gps(self, H, gps1, gps2):
        # Example adjustment based on GPS coordinates
        lat_diff = gps2[0] - gps1[0]
        lon_diff = gps2[1] - gps1[1]

        # Placeholder: Apply a simple translation based on GPS difference
        translation_matrix = np.array([
            [1, 0, lon_diff * 1e6],  # Scale factor for lat/lon differences
            [0, 1, lat_diff * 1e6],  # Adjust the scaling factor as needed
            [0, 0, 1]
        ])

        adjusted_H = np.dot(translation_matrix, H)
        return adjusted_H

    def blend_images(self, img1, img2):
        # Basic blending using alpha blending
        alpha = 0.5
        blended_image = cv2.addWeighted(img1, alpha, img2, 1 - alpha, 0)
        return blended_image

    def get_gps_data(self):
        print("Getting GPS data")
        location = self.vehicle.location.global_frame
        return (location.lat, location.lon)

    def get_accel_data(self):
        print("Getting accelerometer data")
        accel = self.vehicle.attitude
        return accel

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
                    image_data = self.capture_image()  # Capture and process image data here
                    if image_data is not None:
                        print("Captured Image")
                        self.process_images_realtime(image_data)
                    else:
                        print("Image not captured.")
            else:
                # Move from right to left
                for point in reversed(row):
                    lat, lon = point
                    self.goto_location(lat, lon, ALTITUDE)
                    image_data = self.capture_image()  # Capture and process image data here
                    if image_data is not None:
                        print("Captured Image")
                        self.process_images_realtime(image_data)
                    else:
                        print("Image not captured.")

    def capture_image(self):
        # Capture image based on time interval
        current_time = time.time()
        print("Image function...")
        if current_time - self.last_capture_time >= CAPTURE_INTERVAL:
            print("Time Lapse")
            self.last_capture_time = current_time
            self.image_callback()
            return self.image_data
        return None

    def return_to_launch(self):
        print("Returning to launch")
        self.vehicle.mode = VehicleMode("RTL")

    def execute_mission(self):
        self.arm_and_takeoff(ALTITUDE)
        grid = self.generate_grid(self.geofence, GRID_SIZE)
        self.serpentine_path(grid)
        self.return_to_launch()

def main(args=None):
    rclpy.init(args=args)
    drone_mission = DroneMission()
    
    try:
        drone_mission.execute_mission()
        rclpy.spin(drone_mission)
    except Exception as e:
        print(e)
    finally:
        drone_mission.return_to_launch()
        drone_mission.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
