import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class RobotController(Node):
    def __init__(self):
        super().__init__('piduino02')

        # Detect and open serial port
        self.port = self.detect_serial_port()
        self.ser = None

        try:
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            self.get_logger().info(f"Serial connection established on {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial device: {e}")
            return

        # Subscribe to sensor data
        self.create_subscription(Range, '/sensor_data', self.sensor_callback, 10)

        # Publisher for movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ✅ Updated thresholds using meters (converted from cm)
        self.min_distance = 22 / 100  # 22 cm → 0.22 meters (Stop & back up)
        self.adjust_distance = 30 / 100  # 30 cm → 0.30 meters (Minor turn)

        self.get_logger().info("🚀 RobotController node is running...")

        # Start moving forward immediately
        self.move_forward()

    def detect_serial_port(self):
        """ Detect available serial ports and return the first valid one. """
        import glob
        possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        return possible_ports[0] if possible_ports else None

    def move_forward(self):
        """ Move forward """
        self.get_logger().info("🟢 Moving forward")
        self.ser.write(b'F')

    def stop_and_reverse(self):
        """ Stop, reverse, and prepare to turn """
        self.get_logger().info("🛑 TOO CLOSE! Stopping and backing up.")
        self.ser.write(b'S')  # Stop
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))
        self.ser.write(b'B')  # Back up
        self.get_logger().info("🔄 Backing up for 300ms")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))

    def turn_left(self, minor=False):
        """ Turn left slightly or fully based on condition """
        if minor:
            self.get_logger().info("↩️ Minor left turn (Obstacle nearby)")
            self.ser.write(b'L')
        else:
            self.get_logger().info("⬅️ Full left turn (Obstacle too close)")
            self.ser.write(b'L')

    def turn_right(self, minor=False):
        """ Turn right slightly or fully based on condition """
        if minor:
            self.get_logger().info("↪️ Minor right turn (Obstacle nearby)")
            self.ser.write(b'R')
        else:
            self.get_logger().info("➡️ Full right turn (Obstacle too close)")
            self.ser.write(b'R')

    def sensor_callback(self, msg: Range):
        """ Process sensor data, convert cm to meters, and decide movement """
        
        # 🔧 Convert cm to meters
        distance = msg.range / 100.0  

        # ✅ Debugging: Print the original and converted values
        self.get_logger().info(f"📝 Raw Sensor Data: {msg.range:.2f} cm → Converted: {distance:.2f} m")

        if distance < self.min_distance:
            self.get_logger().info("⚠️ TOO CLOSE! Executing stop and reverse.")
            self.stop_and_reverse()
            self.turn_right()
        elif self.min_distance <= distance < self.adjust_distance:
            self.get_logger().info("⚠️ Obstacle nearby, adjusting direction.")
            self.turn_left(minor=True)
        else:
            self.get_logger().info("✅ Path is clear. Moving forward.")
            self.move_forward()

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
