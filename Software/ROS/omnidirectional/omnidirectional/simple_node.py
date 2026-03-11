import rclpy 
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__("simple_node")
        self.cont = 0
        self.get_logger().info("Nodo activo")
        self.create_timer(0.1, self.counter_callback)

    def counter_callback(self):
        self.get_logger().info("Contador: = " + str(self.cont))
        self.cont += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()