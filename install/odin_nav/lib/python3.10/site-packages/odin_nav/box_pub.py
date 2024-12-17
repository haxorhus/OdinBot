#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/odin/box',
            self.goal_callback,
            10)
        self.navigator = BasicNavigator()
        self.get_logger().info('Goal Publisher Node has been started.')

    def goal_callback(self, msg):
        self.get_logger().info(f'Received goal at x: {msg.pose.position.x}, y: {msg.pose.position.y}')
        
        # Set the goal pose with the received message
        goal_pose = PoseStamped()
        goal_pose.header = msg.header
        goal_pose.pose = msg.pose
        
        self.navigator.goToPose(goal_pose)

        # Monitor the task completion
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters.')
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
