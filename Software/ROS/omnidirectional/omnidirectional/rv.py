import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time

class Odometria_sim(Node):
	def __init__(self):
		super().__init__('Odometria_sim')

		self.r = 0.029
		self.L = 0.115

		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.last_time = self.get_clock().now()

		self.wheel_positions = [0.0, 0.0, 0.0]
		self.wheel_velocities = [0.0, 0.0, 0.0]
		#self.subscription_ruedasref = self.create_subscription(Float32MultiArray, '/velocity_controller/commands', self.w_callback, 10)
		self.subscription_ruedasref = self.create_subscription(Float32MultiArray, 'vel_ruedas', self.w_callback, 10)

		self.odom_pub = self.create_publisher(Odometry, 'odom_cin', 10)
		self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

		self.tf_broadcaster = TransformBroadcaster(self)
		self.create_timer(0.01, self.update_odometry)

	def w_callback(self, msg):
		self.wheel_velocities = msg.data

	def update_odometry(self):
		now = self.get_clock().now()
		dt = (now - self.last_time).nanoseconds*1e-9
		self.last_time = now

		w1, w2, w3 = self.wheel_velocities

		vel_x = self.r*(w1 - w3) / math.sqrt(3)
		vel_y = self.r*(-w1 + 2*w2 - w3) / 3
		vel_w = -self.r*(w1 + w2 + w3) / (3 * self.L)
		
		self.x += (vel_x * math.cos(self.theta) - vel_y * math.sin(self.theta)) * dt
		self.y += (vel_x * math.sin(self.theta) + vel_y * math.cos(self.theta)) * dt

		self.theta += vel_w * dt

		odom_msg = Odometry()
		odom_msg.header.stamp = now.to_msg()
		odom_msg.header.frame_id = 'odom'
		odom_msg.child_frame_id = 'base_link'

		odom_msg.pose.pose.position.x = self.x
		odom_msg.pose.pose.position.y = self.y
		odom_msg.pose.pose.position.z = 0.0
		q = self.yaw_to_quaternion(self.theta)
		odom_msg.pose.pose.orientation = q

		odom_msg.twist.twist.linear.x = vel_x
		odom_msg.twist.twist.linear.y = vel_y
		odom_msg.twist.twist.angular.z = vel_w

		self.odom_pub.publish(odom_msg)
		self.get_logger().debug(f'Odometria publicada')

		t = TransformStamped()
		t.header.stamp = now.to_msg()
		t.header.frame_id = 'odom'
		t.child_frame_id = 'base_link'
		t.transform.translation.x = self.x
		t.transform.translation.y = self.y
		t.transform.translation.z = 0.0
		t.transform.rotation = q

		#self.tf_broadcaster.sendTransform(t)

		self.wheel_positions[0] += w1 * dt
		self.wheel_positions[1] += w2 * dt
		self.wheel_positions[2] += w3 * dt
		
		joint_msg = JointState()
		joint_msg.header.stamp = now.to_msg()
		joint_msg.name = ['joint1', 'joint2', 'joint3']
		joint_msg.position = self.wheel_positions
		self.joint_pub.publish(joint_msg)

	def yaw_to_quaternion(self, yaw):
		from geometry_msgs.msg import Quaternion
		q = Quaternion()
		q.w = math.cos(yaw / 2.0)
		q.x = 0.0
		q.y = 0.0
		q.z = math.sin(yaw / 2.0)
		return q
		
def main(args=None):
	rclpy.init(args=args)
	node = Odometria_sim()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
    main()
