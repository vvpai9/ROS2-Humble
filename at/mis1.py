import cv2
import numpy as np
import threading
from datetime import datetime
from sensor_msgs.msg import Image
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil
import rclpy
from rclpy.node import Node

class DroneMission(Node):
    def __init__(self):
        super().__init__("drone_mission")
        self.vehicle = connect('127.0.0.1:14550', wait_ready=True)
        self.subscriber = self.create_subscription(Image, "/camera", self.image_callback, 10)
        self.lock = threading.Lock()
        self.interruption_flag = threading.Event()
        self.stop_event = threading.Event()
        self.object_count = 0
        self.h_x, self.h_y = 400, 240  # Center of the frame
        self.groundspeed = 0.25
        self.CAMERA_RESOLUTION = (1280, 720)
        self.home_lat = self.vehicle.location.global_frame.lat
        self.home_lon = self.vehicle.location.global_frame.lon

        self.geofence = [
            (self.home_lat, self.home_lon),  # Bottom-Left
            (self.home_lat, 149.16631139),  # Bottom-Right
            (-35.36190824, 149.16631139),  # Top-Right
            (-35.36190824, self.home_lon)   # Top-Left
        ]
        self.x_divisions = 100
        self.y_divisions = 100
        self.alt = 10
        self.frame = None  # Store the current frame

    def image_callback(self, msg: Image):
        image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        with self.lock:
            self.frame = image_data

    def show_processed_frame(self):
        while not self.stop_event.is_set():
            with self.lock:
                frame = self.frame

            if frame is None:
                continue

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = red_mask1 + red_mask2

            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            largest_contour = max(contours, key=cv2.contourArea, default=None)

            if largest_contour is not None:
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    with open("coordinates.txt", "a") as f:
                        f.write(f"{cx}, {cy}\n")
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    a, b, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(frame, (a, b), (a + w, b + h), (255, 0, 0), 2)
                
                self.interruption_flag.set()  # Signal that a contour has been detected
            
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow('Processed Frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def take_picture(self):
        self.object_count += 1
        with self.lock:
            frame = self.frame
        if frame is not None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"image_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved image: {filename}")
            time.sleep(1)
            self.image_detect = True
    
    def restartcam(self):
        time.sleep(2)
        self.stop_event.clear()
        if self.processed_thread is not None and self.processed_thread.is_alive():
            self.stop_event.set()
            self.processed_thread.join()
            cv2.destroyAllWindows()
        time.sleep(3)
        self.processed_thread = threading.Thread(target=self.show_processed_frame)
        self.processed_thread.start()

    def clearFile(self, num1, num2):
        with open("coordinates.txt", "w") as f:
            f.write(f"{num1}, {num2}\n")

    def readingCoordinates(self):
        with open("coordinates.txt", "r") as file:
            lines = file.readlines()
            if not lines:
                return None, None
            last_line = lines[-1].strip()
            try:
                x, y = map(int, last_line.split(", "))
                return x, y
            except ValueError:
                return None, None

    def generate_grid(self):
        bottom_left, bottom_right, top_right, top_left = self.geofence
        lat_start, lon_start = bottom_left
        lat_end, lon_end = top_right

        lat_steps = int(self.get_distance_metres(LocationGlobalRelative(lat_start, lon_start), LocationGlobalRelative(lat_end, lon_start)) / self.y_divisions)
        lon_steps = int(self.get_distance_metres(LocationGlobalRelative(lat_start, lon_start), LocationGlobalRelative(lat_start, lon_end)) / self.x_divisions)

        grid = []
        for i in range(lat_steps + 1):
            row = []
            for j in range(lon_steps + 1):
                lat = lat_start + (lat_end - lat_start) * (i / lat_steps)
                lon = lon_start + (lon_end - lon_start) * (j / lon_steps)
                row.append((lat, lon))
            grid.append(row)
        return grid

    def serpentine_path_grid(self, grid):
        for i, row in enumerate(grid):
            if i % 2 == 0:
                for point in row:
                    lat, lon = point
                    self.go_to_location(lat, lon)
            else:
                for point in reversed(row):
                    lat, lon = point
                    self.go_to_location(lat, lon)

    def send_ned_velocity(self, vx, vy, vz, duration):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # BITMASK
            0, 0, 0,  # Position
            vx, vy, vz,  # Velocity
            0, 0, 0,  # Acceleration
            0, 0)  # Yaw, yaw_rate

        for _ in range(int(duration)):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def arm_and_takeoff(self, altitude):
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)
        self.vehicle.simple_takeoff(altitude)
        while True:
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def go_to_location(self, lat, lon):
        location = LocationGlobalRelative(lat, lon, self.alt)
        self.vehicle.simple_goto(location, groundspeed=30)
        print(f"Going to location: Lat={lat}, Lon={lon}")

        while True:
            current_location = self.vehicle.location.global_relative_frame
            distance = self.get_distance_metres(current_location, location)
            print(f"Distance to target: {distance:.2f} meters")
            if distance <= 1:
                print("Reached destination")
                break
            time.sleep(1)
            if self.interruption_flag.is_set():
                self.interruption_flag.clear()
                x, y = self.readingCoordinates()
                if x is not None and y is not None:
                    x_dist = self.h_x - x
                    y_dist = self.h_y - y
                    ned_x, ned_y = x_dist / 50.0, y_dist / 50.0
                    self.send_ned_velocity(ned_x, ned_y, 0, 3)
                self.take_picture()
                self.restartcam()
                self.vehicle.simple_goto(location, groundspeed=30)

    def get_distance_metres(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def start_threads(self):
        self.processed_thread = threading.Thread(target=self.show_processed_frame)
        self.navigation_thread = threading.Thread(target=self.run_serpentine_path)

        
        self.navigation_thread.start()
        time.sleep(5)
        self.processed_thread.start()
        print("Threads have started.")

    def run_serpentine_path(self):
        grid = self.generate_grid()
        self.serpentine_path_grid(grid)
        self.restartcam()

    def main(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.vehicle.mode = VehicleMode("RTL")
            self.stop_event.set()
            if self.processed_thread is not None:
                self.processed_thread.join()
            if self.navigation_thread is not None:
                self.navigation_thread.join()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        rclpy.init()
        drone_mission = DroneMission()
        drone_mission.arm_and_takeoff(10)  # 10 meters altitude
        drone_mission.start_threads()
        print("Detection and Navigation Threads started")
        drone_mission.main()
    except KeyboardInterrupt:
        print("Keyboard Interrupt generated. Returning to Launch...")
    finally:
        drone_mission.vehicle.mode = VehicleMode("RTL")
        rclpy.shutdown()
