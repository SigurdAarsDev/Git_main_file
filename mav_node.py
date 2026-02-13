import rclpy
from rclpy.node import Node
from pymavlink import mavutil

class MavlinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')

        # Connect to Pixhawk (change port and baudrate if needed)
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

        self.get_logger().info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to Pixhawk")

        # Only arm once
        self.armed = False

        # Timer fires every second
        self.timer = self.create_timer(1.0, self.send_arm_command)

    def send_arm_command(self):
        if not self.armed:
            # Send ARM command via MAVLink
            self.master.mav.command_long_send(
                self.master.target_system,        # Target system (from heartbeat)
                self.master.target_component,     # Target component
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,  # Confirmation
                1, 0, 0, 0, 0, 0, 0  # param1=1 to arm, rest unused
            )
            self.get_logger().info("Sent ARM command")
            self.armed = True  # Prevent sending again

def main():
    rclpy.init()
    node = MavlinkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
