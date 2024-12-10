import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
from transforms3d.euler import euler2quat


class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        # Crear broadcasters de TF
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_dynamic_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publicadores para los sensores ultrasónicos
        self.ultrasonic_front_pub = self.create_publisher(Range, '/odin/ultrasound/front', 10)
        self.ultrasonic_left_pub = self.create_publisher(Range, '/odin/ultrasound/left', 10)
        self.ultrasonic_right_pub = self.create_publisher(Range, '/odin/ultrasound/right', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odin/odom', 10)

        # Subscripciones
        self.subscription_us_distance = self.create_subscription(Float32MultiArray, '/odin/us_distance', self.us_distance_callback, 10)
        self.subscription_odom = self.create_subscription(Float32MultiArray, '/odin/data', self.data_callback, 10)


    def publish_static_transform(self, parent_frame, child_frame, x, y, z, roll, pitch, yaw):
        """
        Publica una transformación estática entre dos frames.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        # Configurar la traslación
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # Configurar la rotación como un cuaternión
        quat = euler2quat(roll, pitch, yaw)
        transform.transform.rotation.x = quat[1]  # x
        transform.transform.rotation.y = quat[2]  # y
        transform.transform.rotation.z = quat[3]  # z
        transform.transform.rotation.w = quat[0]  # w

        # Publicar la transformación
        self.tf_broadcaster.sendTransform(transform)
    
    def data_callback(self, msg):
        if len(msg.data) != 5:
            self.get_logger().warn('El mensaje /odin/data no tiene 5 valores.')
            return
        # Extraer valores del vector
        vel_lineal = msg.data[0]
        vel_angular = msg.data[1]
        pos_x = msg.data[2]
        pos_y = msg.data[3]
        yaw = msg.data[4]

        # Crear y publicar mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Configurar posición
        odom_msg.pose.pose.position.x = pos_x
        odom_msg.pose.pose.position.y = pos_y
        odom_msg.pose.pose.position.z = 0.0
        quat = euler2quat(0, 0, yaw)  # Convertir yaw a cuaternión
        odom_msg.pose.pose.orientation.x = quat[1]
        odom_msg.pose.pose.orientation.y = quat[2]
        odom_msg.pose.pose.orientation.z = quat[3]
        odom_msg.pose.pose.orientation.w = quat[0]

        # Configurar velocidades
        odom_msg.twist.twist.linear.x = vel_lineal
        odom_msg.twist.twist.angular.z = vel_angular

        self.odom_pub.publish(odom_msg)

        # Publicar todas las transformaciones estáticas
        self.publish_static_transform('base_link', 'base_footprint', 0.0, 0.0, 0.0, 0, 0, 0)
        self.publish_static_transform('base_link', 'drivewhl_l_link', 0.0, 0.1, 0.0, 0, 0, 0)
        self.publish_static_transform('base_link', 'drivewhl_r_link', 0.0, -0.1, 0.0, 0, 0, 0)
        self.publish_static_transform('base_link', 'ultrasonic_front', 0.1, 0.0, 0.0, 0, 0, 0)
        self.publish_static_transform('base_link', 'ultrasonic_left', 0.0707, 0.0707, 0.0, 0, 0, 0.7854)
        self.publish_static_transform('base_link', 'ultrasonic_right', 0.0707, -0.0707, 0.0, 0, 0, -0.7854)

        # Publicar la transformación odom => base_link
        self.publish_transform(pos_x, pos_y, yaw)
    
    def publish_transform(self, x, y, yaw):
        """
        Publica la transformación dinámica odom => base_link.
        """
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        # Configurar traslación
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        # Configurar rotación
        quat = euler2quat(0, 0, yaw)
        transform.transform.rotation.x = quat[1]
        transform.transform.rotation.y = quat[2]
        transform.transform.rotation.z = quat[3]
        transform.transform.rotation.w = quat[0]

        # Publicar la transformación
        self.tf_dynamic_broadcaster.sendTransform(transform)

    def us_distance_callback(self, msg):
        """Callback para procesar las distancias medidas por los sensores ultrasónicos."""
        distances = msg.data  # Vector con las distancias [front, left, right]
        if len(distances) != 3:
            self.get_logger().warn('El mensaje /us_distance no tiene 3 valores.')
            return

        # Crear mensajes de tipo Range para cada sensor
        range_msg_front = self.create_range_msg(distances[1], 'ultrasonic_front', 0.02, 3.0)
        range_msg_left = self.create_range_msg(distances[2], 'ultrasonic_left', 0.02, 3.0)
        range_msg_right = self.create_range_msg(distances[0], 'ultrasonic_right', 0.02, 3.0)

        # Publicar los mensajes en sus tópicos correspondientes
        self.ultrasonic_front_pub.publish(range_msg_front)
        self.ultrasonic_left_pub.publish(range_msg_left)
        self.ultrasonic_right_pub.publish(range_msg_right)

    def create_range_msg(self, distance, frame_id, min_range, max_range):
        """Crea un mensaje Range para un sensor ultrasónico."""
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = frame_id
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.26  # Ajusta según las especificaciones de tu sensor
        range_msg.min_range = min_range
        range_msg.max_range = max_range
        range_msg.range = distance / 100
        return range_msg


def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
