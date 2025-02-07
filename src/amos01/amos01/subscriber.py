import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__('hello_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Hey everyone, "{msg.data}"')

rclpy.init()
node = HelloWorldSubscriber()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

