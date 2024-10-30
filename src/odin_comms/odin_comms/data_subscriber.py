import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class data_subscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.encoder_r = self.create_subscription(Float32MultiArray, '/enc_vel', self.encoder_callback ,10)
        self.us1 = self.create_subscription(Float32MultiArray, '/us_distance', self.us_callback,10)
        self.get_logger().info('Nodo subscriptor iniciado')
        
    def encoder_callback(self,msg):
        self.get_logger().info(f'Encoders = {msg.data.data}')
    def us_callback(self,msg):
        self.get_logger().info(f'Ultrasonidos = {msg.data.data}')         

def main(args=None):
    rclpy.init(args=args)
    node = data_subscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('cerrando nodo')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()