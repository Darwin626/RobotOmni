import rclpy
from rclpy.node import Node
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, TransformStamped
from std_msgs.msg import Header
import tf2_ros

class OdomSerialNode(Node):
    def __init__(self):
        super().__init__('odom_serial_node')

        # Parámetros configurables
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Conectado al puerto {port} a {baud} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"No se pudo abrir el puerto {port}: {e}")
            exit(1)

        # Publisher de odometría
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Broadcaster de TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Variables internas
        self.q_w = self.q_x = self.q_y = self.q_z = 0.0
        self.vel_x = self.vel_y = self.vel_z = 0.0

        # Timer para leer datos (20 Hz)
        self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            if line.startswith("quat"):
                _, w, x, y, z = line.split('\t')
                self.q_w, self.q_x, self.q_y, self.q_z = map(float, (w, x, y, z))

            elif line.startswith("ggWorld"):
                _, gx, gy, gz = line.split('\t')
                self.vel_x, self.vel_y, self.vel_z = map(float, (gx, gy, gz))

            self.publish_odom_and_tf()

        except Exception as e:
            self.get_logger().warn(f"Error leyendo/parsing: {e}")

    def publish_odom_and_tf(self):
        # Mensaje Odometry
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position = Point(0.0, 0.0, 0.0)
        msg.pose.pose.orientation = Quaternion(
            x=self.q_x,
            y=self.q_y,
            z=self.q_z,
            w=self.q_w
        )

        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = self.vel_x
        msg.twist.twist.angular.y = self.vel_y
        msg.twist.twist.angular.z = self.vel_z

        self.odom_pub.publish(msg)

        # Mensaje TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



