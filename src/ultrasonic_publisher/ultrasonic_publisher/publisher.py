import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(String, 'ultrasonic_data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.timer = self.create_timer(0.5, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Publishing: {data}")
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
