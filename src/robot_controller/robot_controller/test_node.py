import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Node has started!')

    def timer_callback(self):
        self.get_logger().info('Hello from test_node!')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

