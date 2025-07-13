import time
import math
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Connect to the Vehicle
print('Connecting to vehicle...')
vehicle = connect('127.0.0.1:14550', wait_ready=True)

# Parameters
ALTITUDE = 15  # Desired altitude in meters
GROUND_SPEED = 8  # Ground speed in m/s
GRID_SIZE = 20  # Size of each grid cell in meters (10m x 10m)
DETECTED_OBJECTS = 0  # Counter for detected objects

home_lat = vehicle.location.global_frame.lat
home_lon = vehicle.location.global_frame.lon

# Define Geofence Corners (latitude, longitude)
geofence = [
    (home_lat, home_lon),  # Bottom-Left
    (home_lat, 149.16631139),  # Bottom-Right
    (-35.36190824, 149.16631139),  # Top-Right
    (-35.36190824, home_lon)   # Top-Left
]

def arm_and_takeoff(altitude):
    print("Performing pre-arm checks...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(altitude)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def goto_location(to_lat, to_lon, alt):
    target_location = LocationGlobalRelative(to_lat, to_lon, alt)
    vehicle.simple_goto(target_location, groundspeed=GROUND_SPEED)

    while vehicle.mode.name == "GUIDED":
        current_location = vehicle.location.global_relative_frame
        distance = get_distance_metres(current_location, target_location)
        print(f"Distance to target: {distance} meters")
        if distance <= 1.5:
            print("Reached target location")
            break
        time.sleep(1)

def generate_grid(geofence, grid_size):
    bottom_left, bottom_right, top_right, top_left = geofence
    lat_start, lon_start = bottom_left
    lat_end, lon_end = top_right

    # Calculate the number of grid cells needed
    lat_steps = int(get_distance_metres(LocationGlobalRelative(lat_start, lon_start), LocationGlobalRelative(lat_end, lon_start)) / grid_size)
    lon_steps = int(get_distance_metres(LocationGlobalRelative(lat_start, lon_start), LocationGlobalRelative(lat_start, lon_end)) / grid_size)

    grid = []
    for i in range(lat_steps + 1):
        row = []
        for j in range(lon_steps + 1):
            lat = lat_start + (lat_end - lat_start) * (i / lat_steps)
            lon = lon_start + (lon_end - lon_start) * (j / lon_steps)
            row.append((lat, lon))
        grid.append(row)
    return grid

def serpentine_path(grid):
    for i, row in enumerate(grid):
        if i % 2 == 0:
            # Move from left to right
            for point in row:
                lat, lon = point
                goto_location(lat, lon, ALTITUDE)
                # detect_objects(lat, lon, ALTITUDE)
        else:
            # Move from right to left
            for point in reversed(row):
                lat, lon = point
                goto_location(lat, lon, ALTITUDE)
                # detect_objects(lat, lon, ALTITUDE)

def detect_objects(last_lat, last_lon, alt):
    global DETECTED_OBJECTS
    # Initialize the camera
    cap = cv2.VideoCapture(0)  # Use 0 for the default camera

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Use HoughCircles to detect circles in the image
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=100, param1=50, param2=30, minRadius=20, maxRadius=100)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            for (x, y, r) in circles:
                # Draw the circle in the output image
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)

                # Draw a rectangle (center) in the image
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                # Save the current location before moving to the detected object
                saved_location = vehicle.location.global_relative_frame

                # Move to the detected object's location
                move_to_detected_object()

                # Capture the image with a timestamp
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                cv2.imwrite(f"circle_detected_{timestamp}.jpg", frame)

                DETECTED_OBJECTS += 1
                print(f"Circle detected at coordinates ({x}, {y}), image saved. Total objects detected: {DETECTED_OBJECTS}")

                # Return to the saved location
                goto_location(saved_location.lat, saved_location.lon, saved_location.alt)

                # Resume the mission
                return

        # Display the resulting frame
        cv2.imshow('Frame', frame)

        # Break the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all windows
    cap.release()
    cv2.destroyAllWindows()

def move_to_detected_object():
    # Simulate the drone moving closer to the detected object
    # For example, we can assume the drone moves a fixed distance towards the object
    time.sleep(2)  # Simulate the movement time

def return_to_launch():
    print("Returning to launch")
    vehicle.mode = VehicleMode("RTL")

# Main mission execution
def execute_mission():
    arm_and_takeoff(ALTITUDE)

    grid = generate_grid(geofence, GRID_SIZE)
    serpentine_path(grid)

    return_to_launch()

    print("Mission complete")
    print(f"Total objects detected: {DETECTED_OBJECTS}")
    vehicle.close()

# Execute the mission
execute_mission()