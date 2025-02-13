import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_publisher')
        self.publisher_ = self.create_publisher(String, 'ultrasonic_data', 10)

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)  # Faster updates
            self.serial_port.reset_input_buffer()  
            self.get_logger().info("Serial connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        self.timer = self.create_timer(0.05, self.read_serial_data)  # Update every 50ms

    def read_serial_data(self):
        if self.serial_port and self.serial_port.in_waiting > 0:
            try:
                raw_data = self.serial_port.read_all().decode('utf-8', errors='ignore').strip()
                data = raw_data.replace("\r", "").replace("\n", "")  # Clean up data

                # Assuming two distance values in the format "distA distB"
                values = data.split()
                if len(values) == 2 and values[0].isdigit() and values[1].isdigit():
                    formatted_data = f"A:{values[0]} B:{values[1]}"
                    self.get_logger().info(formatted_data)

                    msg = String()
                    msg.data = formatted_data
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().warn(f"Ignoring invalid data: {data}")

            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
