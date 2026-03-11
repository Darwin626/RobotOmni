import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

class Odometria_sim(Node):
	def __init__(self):
		super().__init__('Odometria_sim')

		self.subscription_odom = self.create_subscription(Odometry, '/robot_pose/ground_truth', self.update_odometry, 10)
		self.publisher_pose = self.create_publisher(PoseStamped, '/ground_truth_pose', 10)
		self.tf_broadcaster = TransformBroadcaster(self)

	def update_odometry(self, msg: Odometry):
		pose_msg = PoseStamped()
		pose_msg.header = msg.header
		pose_msg.pose = msg.pose.pose
		self.publisher_pose.publish(pose_msg)

		t = TransformStamped()
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = 'odom'
		t.child_frame_id = 'base_link'

		t.transform.translation.x = msg.pose.pose.position.x
		t.transform.translation.y = msg.pose.pose.position.y
		t.transform.translation.z = msg.pose.pose.position.z
		t.transform.rotation = msg.pose.pose.orientation

		self.tf_broadcaster.sendTransform(t)
        
		
def main(args=None):
	rclpy.init(args=args)
	node = Odometria_sim()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
    main()
