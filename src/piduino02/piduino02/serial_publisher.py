import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust the port and baud rate as needed
        self.ultrasonic_publisher = self.create_publisher(String, 'ultrasonic_data', 10)
        self.encoder_publisher = self.create_publisher(String, 'encoder_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith('A:'):
                self.ultrasonic_publisher.publish(String(data=line))
                self.get_logger().info(f'Publishing ultrasonic data: {line}')
            elif line.startswith('Left Encoder:'):
                self.encoder_publisher.publish(String(data=line))
                self.get_logger().info(f'Publishing encoder data: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
     