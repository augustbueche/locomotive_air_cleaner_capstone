import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            String,
            'ultrasonic_data',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'Received data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
