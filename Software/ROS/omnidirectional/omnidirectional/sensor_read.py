import rclpy
from rclpy.node import Node
import serial
import json
import math

from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

class Sensor_read(Node):
    def __init__(self):
        super().__init__('esp32_sensors_node')

        # Publishers
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.pub_linea = self.create_publisher(Float32MultiArray, 'sensors/linea', 10)

        self.pub_dist = self.create_publisher(LaserScan, 'sensors/distancia', 100)
        self.pub_vel = self.create_publisher(Float32MultiArray, 'vel_ruedas', 10)

        # Suscriptor
        self.subs_w = self.create_subscription(Float32MultiArray, '/velocity_controller/commands', self.ser_write, 20)

        # Configuración del puerto serie
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0)

        # Timer para leer puerto serie
        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz

    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    continue

                data_read = json.loads(line)

                # --- IMU ---
                imu_msg = Imu()
                imu_msg.orientation.w = data_read['quat'][0]
                imu_msg.orientation.x = data_read['quat'][1]
                imu_msg.orientation.y = data_read['quat'][2]
                imu_msg.orientation.z = data_read['quat'][3]

                imu_msg.linear_acceleration.x = data_read['accel'][0]
                imu_msg.linear_acceleration.y = data_read['accel'][1]
                imu_msg.linear_acceleration.z = data_read['accel'][2]

                imu_msg.angular_velocity.x = data_read['gyro'][0]
                imu_msg.angular_velocity.y = data_read['gyro'][1]
                imu_msg.angular_velocity.z = data_read['gyro'][2]

                self.pub_imu.publish(imu_msg)

                # --- Sensores de línea ---
                linea_msg = Float32MultiArray()
                linea_msg.data = data_read['linea']
                self.pub_linea.publish(linea_msg)

                # --- Sensores de distancia ---
                dist_msg = LaserScan()
                dist_msg.header.stamp = self.get_clock().now().to_msg()
                dist_msg.header.frame_id = 'sharp_array_link'
                #dist_msg.angle_max = math.pi
                #dist_msg.angle_min = -math.pi
                dist_msg.angle_min = 0.0
                dist_msg.angle_max = 2*math.pi

                dist_msg.range_min = 0.12
                dist_msg.range_max = 0.80
                dist_msg.angle_increment  = 2*math.pi/9
                dist_msg.scan_time = 0.05
                dist_msg.ranges = data_read['dist_cm']
                self.pub_dist.publish(dist_msg)
                
                # --- Sensores de velocidad ---
                vel_msg = Float32MultiArray()

                vel_msg.data = data_read['vel']

                vel_msg.data[0] = vel_msg.data[0] * (2*math.pi) / 60.0
                vel_msg.data[1] = vel_msg.data[1] * (2*math.pi) / 60.0
                vel_msg.data[2] = vel_msg.data[2] * (2*math.pi) / 60.0

                self.pub_vel.publish(vel_msg)

        except json.JSONDecodeError:
            self.get_logger().warn("Error parseando JSON")

        except Exception as e:
            self.get_logger().error(f"Error leyendo serial: {e}")

    def ser_write(self, msg):
        # --- Conoversion de [rad/s] a [rpm] ----
        msg_rpm1 = msg.data[0] * 60.0 / (2*math.pi)
        msg_rpm2 = msg.data[1] * 60.0 / (2*math.pi)
        msg_rpm3 = msg.data[2] * 60.0 / (2*math.pi)
        
        # --- Pasar a String ---
        msg_str = f"{msg_rpm1:.3f},{msg_rpm2:.3f},{msg_rpm3:.3f}\n"

        self.get_logger().info(f'Enviado por serial: {msg_str}')

        # --- Enviar por puerto serial ---
        self.ser.write(msg_str.encode())

def main(args=None):
    rclpy.init(args=args)
    node = Sensor_read()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
