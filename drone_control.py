import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time

class DroneControl(Node):

    def __init__(self):
        super().__init__('drone_control')
        self.connection = mavutil.mavlink_connection('udpin:localhost:14540')
        self.connection.wait_heartbeat()

    def arm_drone(self):
        self.connection.mav.command_long_send(
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
        self.get_logger().info(f"Drone armed: {msg}")

    def takeoff(self, altitude):
        self.connection.mav.command_long_send(
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
        self.get_logger().info(f"Drone took off: {msg}")

    def land(self):
        self.connection.mav.command_long_send(
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
        self.get_logger().info(f"Drone landed: {msg}")

    def run(self):
        self.arm_drone()
        time.sleep(2)
        self.takeoff(10)
        time.sleep(10)
        self.land()

def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

