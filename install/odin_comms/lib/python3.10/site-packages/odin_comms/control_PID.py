import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('control_motor')
        
        # Publicadores para cada motor
        self.publisher_motor_r = self.create_publisher(Int32, '/cmd_vel_r', 10)
        self.publisher_motor_l = self.create_publisher(Int32, '/cmd_vel_l', 10)
        
        # Subscripción al valor de los encoders derecho e izquierdo
        self.subscription_encoders = self.create_subscription(Float32MultiArray, '/enc_vel', self.encoder_callback, 10)
        
        # Parámetros del PID
        self.kp = 6.096  # Ganancia proporcional
        self.ki = 4.064  # Ganancia integral
        self.kd = 6.035  # Ganancia derivativa
        
        # Variables PID para ambos motores
        self.previous_error_r = 0
        self.integral_r = 0
        self.previous_error_l = 0
        self.integral_l = 0
        self.desired_velocity_rpm_r = 0
        self.desired_velocity_rpm_l = 0
        self.current_velocity_rpm_r = 0
        self.current_velocity_rpm_l = 0
        self.previous_time = time.time()

        # Parámetros del motor
        self.max_rpm = 170  # Cambia este valor si el RPM máximo del motor cambia
        self.max_pwm = 255  # Rango de PWM (-255 a 255)

        self.get_logger().info('Nodo control de motores con PID iniciado')
    
    def encoder_callback(self, msg):
        # Actualizar las velocidades actuales de los encoders
        if len(msg.data) >= 2:
            self.current_velocity_rpm_r = msg.data[0]
            self.current_velocity_rpm_l = msg.data[1]
            self.get_logger().info(f'Velocidad actual (RPM) - Motor Derecho: {self.current_velocity_rpm_r}, Motor Izquierdo: {self.current_velocity_rpm_l}')
            self.pid_control()
    
    def pid_control(self):
        # Calcular el tiempo desde la última actualización
        current_time = time.time()
        delta_time = current_time - self.previous_time

        # Control PID para el motor derecho
        error_r = self.desired_velocity_rpm_r - self.current_velocity_rpm_r
        self.integral_r += error_r * delta_time
        derivative_r = (error_r - self.previous_error_r) / delta_time if delta_time > 0 else 0
        output_rpm_r = self.kp * error_r + self.ki * self.integral_r + self.kd * derivative_r
        output_pwm_r = self.rpm_to_pwm(output_rpm_r)
        self.publish_pwm(output_pwm_r, 'r')
        self.previous_error_r = error_r

        # Control PID para el motor izquierdo
        error_l = self.desired_velocity_rpm_l - self.current_velocity_rpm_l
        self.integral_l += error_l * delta_time
        derivative_l = (error_l - self.previous_error_l) / delta_time if delta_time > 0 else 0
        output_rpm_l = self.kp * error_l + self.ki * self.integral_l + self.kd * derivative_l
        output_pwm_l = self.rpm_to_pwm(output_rpm_l)
        self.publish_pwm(output_pwm_l, 'l')
        self.previous_error_l = error_l

        # Actualizar el tiempo para la siguiente iteración
        self.previous_time = current_time
    
    def rpm_to_pwm(self, rpm):
        """Convierte el valor de RPM a PWM (-255 a 255)."""
        pwm = (rpm / self.max_rpm) * self.max_pwm
        return max(min(pwm, self.max_pwm), -self.max_pwm)  # Limitar a -255 y 255

    def publish_pwm(self, pwm_value, motor_side):
        """Publica el valor de PWM en el tópico correspondiente."""
        msg = Int32()
        msg.data = int(pwm_value)
        if motor_side == 'r':
            self.publisher_motor_r.publish(msg)
            self.get_logger().info(f'Publicando PWM Motor Derecho: {pwm_value} en el topic /cmd_vel_r')
        elif motor_side == 'l':
            self.publisher_motor_l.publish(msg)
            self.get_logger().info(f'Publicando PWM Motor Izquierdo: {pwm_value} en el topic /cmd_vel_l')

    def main_loop(self):
        try:
            while rclpy.ok():
                # Leer las velocidades deseadas en RPM desde el teclado
                input_value_r = input("Ingrese la velocidad deseada para el motor derecho (RPM, entre -170 y 170): ")
                input_value_l = input("Ingrese la velocidad deseada para el motor izquierdo (RPM, entre -170 y 170): ")
                try:
                    self.desired_velocity_rpm_r = int(input_value_r)
                    self.desired_velocity_rpm_l = int(input_value_l)
                    
                    # Validar que las velocidades estén dentro del rango permitido
                    if abs(self.desired_velocity_rpm_r) > self.max_rpm or abs(self.desired_velocity_rpm_l) > self.max_rpm:
                        self.get_logger().error(f'Las velocidades deben estar entre -{self.max_rpm} y {self.max_rpm}')
                        continue
                except ValueError:
                    self.get_logger().error('Valor de velocidad no válido')
                    continue
                
                # Esperar un poco antes de la siguiente iteración
                time.sleep(0.1)
        except KeyboardInterrupt:
            self.get_logger().info('Cerrando nodo')
        finally:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    node.main_loop()

if __name__ == '__main__':
    main()
