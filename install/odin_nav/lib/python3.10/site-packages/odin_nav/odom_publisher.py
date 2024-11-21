import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
import math
import time

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        # Suscripción a los datos de velocidad del encoder
        self.enc_vel_sub = self.create_subscription(Float32MultiArray, '/enc_vel', self.encoder_callback, 10)
        # Publicador de la odometría
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Parámetros del robot
        self.wheel_base = 0.5  # Distancia entre las ruedas (en metros)
        self.wheel_radius = 0.1  # Radio de las ruedas (en metros)
        
        # Variables de posición y orientación
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def encoder_callback(self, msg):
        # Asumiendo que el arreglo Float32MultiArray tiene la velocidad de cada rueda: [vel_izquierda, vel_derecha]
        vel_left, vel_right = msg.data

        # Calcular la velocidad lineal y angular del robot
        linear_velocity = (vel_right + vel_left) * self.wheel_radius / 2.0
        angular_velocity = (vel_right - vel_left) * self.wheel_radius / self.wheel_base

        # Calcular el tiempo transcurrido
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Actualizar la posición y orientación del robot
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Crear y publicar el mensaje de odometría
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Posición
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientación en forma de cuaternión
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # Velocidades lineal y angular
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        # Publicar el mensaje de odometría
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f'Odom published: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
