import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
from transforms3d.euler import quat2euler, euler2quat

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Suscripción a /enc_vel y /odin/robot_position
        self.encoder_sub = self.create_subscription(Float32MultiArray, '/enc_vel', self.encoder_callback, 10)
        self.position_sub = self.create_subscription(PoseStamped, '/odin/robot_position', self.position_callback, 10)

        # Variables de odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.initialized = False  # Para verificar si la posición inicial ya fue recibida

        # Parámetros del robot
        self.wheel_radius = 0.0326  # Radio de las ruedas en metros
        self.wheel_base = 0.2  # Distancia entre las ruedas en metros
        self.dt = 1.0  # Intervalo de tiempo de actualización en segundos

        self.get_logger().info('Nodo de publicación de odometría y transformadas iniciado')

    def position_callback(self, msg):
        if not self.initialized:
            # Configurar la posición inicial desde /odin/robot_position
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            _, _, self.theta = quat2euler([
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ])
            self.initialized = True
            self.get_logger().info(f'Posición inicial establecida en x={self.x}, y={self.y}, theta={self.theta:.3f} rad')

    def encoder_callback(self, msg):
        if not self.initialized:
            self.get_logger().warn('Posición inicial aún no recibida. Esperando...')
            return

        if len(msg.data) != 2:
            self.get_logger().warn('Datos de /enc_vel no válidos')
            return

        rpm_left = msg.data[1] /2 
        rpm_right = msg.data[0] /2

        # Convertir RPM a velocidad angular (rad/s)
        omega_left = (rpm_left / 60.0) * 2 * math.pi
        omega_right = (rpm_right / 60.0) * 2 * math.pi

        # Convertir a velocidades lineales (m/s)
        vel_left = omega_left * self.wheel_radius
        vel_right = omega_right * self.wheel_radius

        # Calcular velocidades del robot
        vel_linear = (vel_left + vel_right) / 2.0
        vel_angular = (vel_right - vel_left) / self.wheel_base

        # Calcular la nueva posición
        delta_x = vel_linear * math.cos(self.theta) * self.dt
        delta_y = vel_linear * math.sin(self.theta) * self.dt
        delta_theta = vel_angular * self.dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Convertir theta a quaternion
        quat = euler2quat(0, 0, self.theta)

        # Publicar odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]
        odom_msg.pose.pose.orientation.w = quat[0]
        odom_msg.twist.twist.linear.x = vel_linear
        odom_msg.twist.twist.angular.z = vel_angular

        self.odom_pub.publish(odom_msg)

        # Publicar transformada odom => base_link
        transform = TransformStamped()
        transform.header.stamp = odom_msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        transform.transform.rotation.w = quat[0]

        self.tf_broadcaster.sendTransform(transform)

        # Publicar transformada base_link => base_footprint
        self.publish_static_transform('base_link', 'base_footprint', 0.0, 0.0, 0.0, 0, 0, 0)

        # Publicar transformadas base_link => drivewhl_l_joint y base_link => drivewhl_r_joint
        self.publish_static_transform('base_link', 'drivewhl_l_joint', 0.0, 0.1, 0.0, 0, 0, 0)
        self.publish_static_transform('base_link', 'drivewhl_r_joint', 0.0, -0.1, 0.0, 0, 0, 0)

    def publish_static_transform(self, parent_frame, child_frame, x, y, z, roll, pitch, yaw):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        quat = euler2quat(roll, pitch, yaw)
        transform.transform.rotation.x = quat[1]  # x
        transform.transform.rotation.y = quat[2]  # y
        transform.transform.rotation.z = quat[3]  # z
        transform.transform.rotation.w = quat[0]  # w
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando nodo de odometría')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
