import rclpy  # Add this import
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'amos says hi'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

rclpy.init()
node = HelloWorldPublisher()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()

