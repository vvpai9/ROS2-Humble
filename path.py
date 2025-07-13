import time
import math
from collections import OrderedDict
from pymavlink import mavutil
import numpy as np
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import argparse
import matplotlib.pyplot as plt

# Define field dimensions and drone parameters
field_length = 100  # x dimension in meters
field_width = 50    # y dimension in meters
drone_height = 15   # height in meters
fov = 60            # field of view in degrees

# Calculate ground coverage width
half_angle = fov / 2
half_width = drone_height * math.tan(math.radians(half_angle))
coverage_width = 2 * half_width
#print(coverage_width)
# Generate optimized path that doesn't exceed field boundaries
def generate_optimized_path(field_length, field_width, coverage_width):
    path = []
    y = 0
    direction = 1
    
    while y < field_width:
        start_x = 0 if direction > 0 else field_length - coverage_width / 2
        end_x = field_length if direction > 0 else coverage_width / 2
        
        path.append((start_x, y))
        path.append((end_x, y))
        
        y += coverage_width
        direction *= -1
    
    # Ensure the last strip is covered if the field width is not an exact multiple of the coverage width
    if y < field_width:
        start_x = 0 if direction > 0 else field_length - coverage_width / 2
        end_x = field_length if direction > 0 else coverage_width / 2
        path.append((start_x, field_width))
        path.append((end_x, field_width))
    
    return path

def path_create():
    # Calculate the path
    path = generate_optimized_path(field_length, field_width, coverage_width)

    # Print the path
    for waypoint in path:
        print(f"Move to {waypoint}")

    # Visualize the path using matplotlib
    x_coords, y_coords = zip(*path)
    

# Argument parser for connection string
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True, timeout=60)

temp_dict = {'OrderedDict': OrderedDict}
global x
global y
global clas
global hotcount
global detections
global in_x
global in_y
x = 0
y = 0
clas = 'v'
CAMERA_RESOLUTION = (1280, 720)
groundspeed = 0.25
cam_y = 17.129568167795526  # in meters
cam_x = 40.01904440518039
short = []


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def goto_location_norm(to_lat, to_lon, alt):
    currentLocation = vehicle.location.global_frame
    target_Location = LocationGlobalRelative(to_lat, to_lon, alt)
    targetDistance = get_distance_metres(currentLocation, target_Location)
    Distance = get_distance_metres(vehicle.location.global_frame, target_Location)
    vehicle.simple_goto(target_Location, groundspeed=2)

    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(vehicle.location.global_frame, target_Location)
        print("Distance to waypoint: ", remainingDistance)
        if remainingDistance <= 1:  # Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b10111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_y, velocity_x, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_ned_velocity1(velocity_z, to_alt):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        0, 0, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    if to_alt == 0:
        vehicle.send_mavlink(msg)
        time.sleep(1)
        return

    while True:
        vehicle.send_mavlink(msg)
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= to_alt - 1 and vehicle.location.global_relative_frame.alt <= to_alt + 1:
            print("Reached target altitude")
            break
        time.sleep(1)



# def mission():
#     alt = 5
#     print(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
#     print(vehicle.heading)

#     arm_and_takeoff(alt)  # Take off to 5m

#     # Fly the drone to each waypoint in the optimized path
#     for waypoint in path:
#         to_lat, to_lon = waypoint
#         goto_location_norm(to_lat, to_lon, alt)
#         time.sleep(2)

#     # print("Returning to Launch")
#     # vehicle.mode = VehicleMode("RTL")

#     # print("Mission complete")
#     # vehicle.mode = VehicleMode("LAND")

# mission()

# vehicle.close()

def mission():
    alt = 5
    print(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon)
    print(vehicle.heading)

    arm_and_takeoff(alt)  # Take off to 5m

    # Create a list to store the waypoints
    waypoints = []

    # Fly the drone to each waypoint in the optimized path
    path = generate_optimized_path(field_length, field_width, coverage_width)
    for point in path:
        x, y = point
        print(x,y)
        #to_lat = (x / num_cells_width) * field_length
        #to_lon = (y / num_cells_height) * field_width
        #waypoints.append((to_lat, to_lon))  # Store the waypoint coordinates
        waypoints.append((x, y))  # Store the waypoint coordinates
        send_ned_velocity(x, y, 0)
        time.sleep(2)

    # Print the list of waypoints
    print("Waypoints:")
    for i, waypoint in enumerate(waypoints):
        print(f"Waypoint {i + 1}: Latitude={waypoint[0]}, Longitude={waypoint[1]}")
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    # Close the vehicle connection
    vehicle.close()

# Call the mission function
mission()
