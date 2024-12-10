#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class MultiGoalPublisher(Node):
    def __init__(self):
        super().__init__('multi_goal_publisher_node')
        
        self.navigator = BasicNavigator()
        self.get_logger().info('Multi Goal Publisher Node has been started.')
        
        self.box_goal = None
        self.final_goal = None
        
        # Suscribirse a los tópicos
        self.box_subscription = self.create_subscription(
            PoseStamped,
            '/odin/box',
            self.box_callback,
            10)

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/odin/goal',
            self.goal_callback,
            10)

    def box_callback(self, msg):
        self.get_logger().info(f'Received box goal at x: {msg.pose.position.x}, y: {msg.pose.position.y}')
        self.box_goal = msg
        self.navigate_to_goal(self.box_goal, "Box")

    def goal_callback(self, msg):
        self.get_logger().info(f'Received final goal at x: {msg.pose.position.x}, y: {msg.pose.position.y}')
        self.final_goal = msg
        if self.box_goal:
            self.navigate_to_goal(self.final_goal, "Final")
        else:
            self.get_logger().info('Waiting for box goal to complete before navigating to final goal.')

    def navigate_to_goal(self, goal_msg, goal_name):
        self.get_logger().info(f'Navigating to {goal_name} goal...')
        
        # Configurar el objetivo de navegación
        goal_pose = PoseStamped()
        goal_pose.header = goal_msg.header
        goal_pose.pose = goal_msg.pose
        
        self.navigator.goToPose(goal_pose)

        # Monitorizar la tarea
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance remaining to {goal_name}: {feedback.distance_remaining:.2f} meters.')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'{goal_name} goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info(f'{goal_name} goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info(f'{goal_name} goal failed!')
        else:
            self.get_logger().info(f'{goal_name} goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
