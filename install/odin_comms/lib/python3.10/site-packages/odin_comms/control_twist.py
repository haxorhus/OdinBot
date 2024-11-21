import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class TwistToMotorControlNode(Node):
    def __init__(self):
        super().__init__('twist_to_motor_control')
        
        # Publicadores para los motores derecho e izquierdo
        self.publisher_motor_r = self.create_publisher(Int32, '/cmd_vel_r', 10)
        self.publisher_motor_l = self.create_publisher(Int32, '/cmd_vel_l', 10)
        
        # Subscripción al tópico /cmd_vel
        self.subscription_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Parámetros del motor
        self.max_rpm = 130  # Cambia este valor si el RPM máximo del motor cambia
        self.max_pwm = 255  # Rango de PWM (-255 a 255)
        self.wheel_base = 0.2  # Distancia entre las ruedas en metros (ajusta según tu robot)

        self.get_logger().info('Nodo Twist a control de motores iniciado')
    
    def cmd_vel_callback(self, msg):
        # Extraer componentes lineal y angular del mensaje Twist
        linear_velocity = msg.linear.x  # Velocidad lineal en m/s
        angular_velocity = msg.angular.z  # Velocidad angular en rad/s

        # Convertir velocidades lineal y angular a velocidades de rueda (RPM)
        rpm_right = self.linear_angular_to_rpm(linear_velocity, angular_velocity, motor='right')
        rpm_left = self.linear_angular_to_rpm(linear_velocity, angular_velocity, motor='left')

        # Convertir RPM a PWM
        # rpm*pwmMAX   + pwmMAX
        pwm_right = int((rpm_right / 170) * 255)
        pwm_left = int((rpm_left / 130) * 255)

        # Publicar los valores de PWM en los tópicos respectivos
        self.publish_pwm(pwm_right, 'r')
        self.publish_pwm(pwm_left, 'l')

    def linear_angular_to_rpm(self, linear_velocity, angular_velocity, motor):
        """Convierte velocidad lineal y angular a RPM para cada motor."""
        if motor == 'right':
            wheel_velocity = linear_velocity + (angular_velocity * self.wheel_base / 2)
        elif motor == 'left':
            wheel_velocity = linear_velocity - (angular_velocity * self.wheel_base / 2)
        
        # Convertir velocidad de m/s a RPM
        rpm = (wheel_velocity * 60) / (2 * 3.14159 * 0.1)  # Suponiendo un radio de rueda de 0.1 m
        return max(min(rpm, self.max_rpm), -self.max_rpm)  # Limitar a -max_rpm y max_rpm

    def rpm_to_pwm(self, rpm):
        """Convierte el valor de RPM a PWM (-255 a 255)."""
        pwm = (rpm / self.max_rpm) * self.max_pwm
        return int(max(min(pwm, self.max_pwm), -self.max_pwm))  # Limitar a -255 y 255

    def publish_pwm(self, pwm_value, motor_side):
        """Publica el valor de PWM en el tópico correspondiente."""
        msg = Int32()
        msg.data = pwm_value
        if motor_side == 'r':
            self.publisher_motor_r.publish(msg)
            #self.get_logger().info(f'Publicando PWM Motor Derecho: {pwm_value} en el topic /cmd_vel_r')
        elif motor_side == 'l':
            self.publisher_motor_l.publish(msg)
            #self.get_logger().info(f'Publicando PWM Motor Izquierdo: {pwm_value} en el topic /cmd_vel_l')

def main(args=None):
    rclpy.init(args=args)
    node = TwistToMotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
