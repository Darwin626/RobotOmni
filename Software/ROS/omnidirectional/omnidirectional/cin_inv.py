import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
#import serial
import math

class Cinematica_inversa(Node):
    def __init__(self):
        super().__init__("Cinematica_inversa")

        self.r = 0.029
        self.L = 0.115

        self.subscription_cmd = self.create_subscription(Twist, '/cmd_vel', self.cin_inv, 10)
        self.publisher_vel = self.create_publisher(Float32MultiArray, '/velocity_controller/commands', 10)

    def cin_inv(self, msg):
        vel_x = msg.linear.x
        vel_y = msg.linear.y
        vel_w = msg.angular.z
        
        w1 = (math.sqrt(3)*vel_x/2 - vel_y/2 - self.L*vel_w)/(self.r)
        w2 = (vel_y - self.L*vel_w)/(self.r)
        w3 = (-math.sqrt(3)*vel_x/2 - vel_y/2 - self.L*vel_w)/(self.r)
        
        vel = Float32MultiArray()
        vel.data = [w1, w2, w3]
        self.publisher_vel.publish(vel)
        
        self.get_logger().info(f'w1: {w1:.2f}, w2: {w2:.2f}, w3: {w3:.2f}, vel_x: {vel_x:.2f}, vel_y: {vel_y:.2f}, vel_w: {vel_w:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = Cinematica_inversa()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
