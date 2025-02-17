import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class RobotController(Node):
    def __init__(self):
        super().__init__('piduino02')

        # Initialize serial connection
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            return

        # Subscribe to cmd_vel topic
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for sensor data
        self.sensor_pub = self.create_publisher(Range, '/sensor_data', 10)

        # Timer to read from serial
        self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                data = self.ser.readline().decode('utf-8').strip()
                if data.startswith("DIST"):
                    _, distanceA, distanceB = data.split()
                    msg = Range()
                    msg.range = min(float(distanceA), float(distanceB)) / 100.0  # cm to meters
                    self.sensor_pub.publish(msg)
                    self.get_logger().info(f"Published sensor data: {msg.range} m")
            except Exception as e:
                self.get_logger().warn(f"Error reading sensor data: {e}")

    def cmd_vel_callback(self, msg: Twist):
        if msg.linear.x > 0:
            self.ser.write(b'F')  # Forward
        elif msg.linear.x < 0:
            self.ser.write(b'B')  # Backward
        elif msg.angular.z > 0:
            self.ser.write(b'L')  # Left
        elif msg.angular.z < 0:
            self.ser.write(b'R')  # Right
        else:
            self.ser.write(b'S')  # Stop

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
