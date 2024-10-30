import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MotorControlNode2(Node):
    def __init__(self):
        super().__init__('control_motor_izquierdo')
        self.publisher_motor_l = self.create_publisher(Int32,'/cmd_vel_l',10)
        self.get_logger().info('Nodo control de motor izquierdo iniciado')

    def publish_vel(self,velocity):
        msg = Int32()
        msg.data = velocity
        self.publisher_motor_l.publish(msg)
        self.get_logger().info('Publicando: {velocity} en el topic')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode2()

    try:
        while rclpy.ok():
            input_value = input("ingrese el valor de velocidad")
            try:
                velocity = int(input_value)
                node.publish_vel(velocity)
            except ValueError:
                node.get_logger().error('Error')
    except KeyboardInterrupt:
        node.get_logger().info('cerrando nodo')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()