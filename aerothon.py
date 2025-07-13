from dronekit import connect, VehicleMode, LocationGlobalRelative
from std_msgs.msg import Int32MultiArray
import rclpy
from rclpy.node import Node
import time
import sys

class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        self.vehicle = None
        self.coordinates_publisher = self.create_publisher(Int32MultiArray, "detection_coordinates", 10)
        self.x_coordinate = None
        self.y_coordinate = None

    def arm_and_takeoff(self, aTargetAltitude):
        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print("Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        
        while not self.vehicle.mode == 'GUIDED':
            print("Waiting for mode change...", self.vehicle.mode.name)
            time.sleep(1)

        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        print("Mode:", self.vehicle.mode.name)
        self.vehicle.simple_takeoff(aTargetAltitude)

        while True:
            print("Altitude:", self.vehicle.location.global_relative_frame.alt)
            if not self.vehicle.armed:
                print("Disarmed")
                break
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def goto_location(self, to_lat, to_lon, to_alt):
        print("Going to location (lat: {}, lon: {}, alt: {})".format(to_lat, to_lon, to_alt))
        point = LocationGlobalRelative(to_lat, to_lon, to_alt)
        self.vehicle.simple_goto(point)

    def land_at_coordinates(self, to_lat, to_lon):
        print("Landing at coordinates (lat: {}, lon: {})".format(to_lat, to_lon))
        point = LocationGlobalRelative(to_lat, to_lon, 0)
        self.vehicle.mode = VehicleMode("LAND")
        self.vehicle.simple_goto(point)

    def publish_coordinates(self, lat, lon):
        msg = Int32MultiArray()
        msg.data = [int(lat), int(lon)]
        self.coordinates_publisher.publish(msg)

    def delay_one_second(delay):
        time.sleep(delay)


def main():
    rclpy.init()
    node = MissionNode()

    try:
        # Connect to the vehicle
        node.vehicle = connect('127.0.0.1:14550', baud=57600, wait_ready=True, timeout=120)
        # Arm and take off
        node.arm_and_takeoff(10)
        
        node.publish_coordinates(-35.36352119, 149.16635865)
        node.goto_location(-35.36352119, 149.16635865, 20)
        time.sleep(20)
        node.publish_coordinates(-35.36190824 ,149.16631139)
        node.goto_location(-35.36190824, 149.16631139, 20)
        time.sleep(30)
        
        node.vehicle.mode = VehicleMode("RTL")
        
        rclpy.spin(node)
        
        if node.vehicle.mode == 'RTL':
            print("Mission completed, returning to launch")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit()
    except KeyboardInterrupt:
        print("Keyboard interrupt received, changing mode to RTL")
        if node.vehicle:
            node.vehicle.mode = VehicleMode("RTL")
            while node.vehicle.mode != 'RTL':
                print("Waiting for mode change to RTL...", node.vehicle.mode.name)
                time.sleep(1)
    except Exception as e:
        print("Mission failed:", e)
    finally:
        if node.vehicle:
            print("Closing vehicle object")
            node.vehicle.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

