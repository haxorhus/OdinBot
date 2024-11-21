#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
import cv2
import numpy as np
import tf2_ros
from cv_bridge import CvBridge
import os

class MapProcessorNode(Node):
    def __init__(self):
        super().__init__('map_processor_node')

        # Publisher for the processed map and positions
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/demo/goal', 10)
        self.robot_position_pub = self.create_publisher(PoseStamped, '/demo/robot_position', 10)
        
        # TF broadcaster for static transform
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Load and process image
        self.image_path = '/home/jose/Downloads/area.jpeg'
        self.map_dir = '/home/jose/microros_ws/src/maps'
        self.bridge = CvBridge()
        self.process_and_save_image()

    def process_and_save_image(self):
        # Read and process image
        image = cv2.imread(self.image_path)
        if image is None:
            self.get_logger().error(f"Failed to load image from {self.image_path}")
            return
        
        # Convert to HSV and process as in camerapavon.py
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # --- DETECCIÓN DE OBJETOS ROJOS ---
        # Rango de color rojo en HSV
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2

        # Process and find contours for binaryred
        binaryred = cv2.bitwise_not(mask_red)  # Invertir para fondo blanco

        # Save binaryred as PGM
        pgm_filename = os.path.join(self.map_dir, 'map.pgm')
        cv2.imwrite(pgm_filename, binaryred)
        self.get_logger().info(f'Map saved as PGM at {pgm_filename}')

        # Save corresponding YAML file
        yaml_filename = os.path.join(self.map_dir, 'map.yaml')
        with open(yaml_filename, 'w') as yaml_file:
            yaml_file.write(f"image: map.pgm\n")
            yaml_file.write("resolution: 0.05\n")
            yaml_file.write("origin: [0.0, 0.0, 0.0]\n")
            yaml_file.write("negate: 0\n")
            yaml_file.write("occupied_thresh: 0.65\n")
            yaml_file.write("free_thresh: 0.196\n")
        self.get_logger().info(f'Map metadata saved as YAML at {yaml_filename}')

        # Publish the map from the saved PGM
        map_msg = self.create_map_msg(binaryred)
        self.map_pub.publish(map_msg)
        self.get_logger().info('Map published successfully')

        # Calculate and publish goal and robot position as PoseStamped
        goal_pose, robot_pose = self.calculate_positions(binaryred)
        self.goal_pub.publish(goal_pose)
        self.robot_position_pub.publish(robot_pose)

        # Publish static transform
        self.publish_static_transform()

    def create_map_msg(self, binary_image):
        # Convert the binary image into an OccupancyGrid message
        height, width = binary_image.shape
        resolution = 0.005  # Adjust as necessary
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        map_msg.info.height = height
        map_msg.info.width = width
        map_msg.info.resolution = resolution
        map_msg.info.origin.position.x = 0.0  # Initial origin
        map_msg.info.origin.position.y = 0.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Convert binary image data to OccupancyGrid data
        map_data = np.where(binary_image > 127, 0, 100).flatten().tolist()  # 0 = free, 100 = occupied
        map_msg.data = map_data

        return map_msg

    def calculate_positions(self, binary_image):
        # Find the centroid of the largest contour (goal) and the center of a circle (robot)
        contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(max_contour)
            if M['m00'] != 0:
                cx_goal = int(M['m10'] / M['m00'])
                cy_goal = int(M['m01'] / M['m00'])
            else:
                cx_goal, cy_goal = 0, 0

            # Create PoseStamped for the goal
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = cx_goal * 0.05  # Convert pixels to meters based on resolution
            goal_pose.pose.position.y = cy_goal * 0.05
            goal_pose.pose.orientation.w = 1.0

            # Simulate robot position as an example
            robot_pose = PoseStamped()
            robot_pose.header.frame_id = 'map'
            robot_pose.header.stamp = self.get_clock().now().to_msg()
            robot_pose.pose.position.x = (cx_goal - 50) * 0.05  # Offset for simulation
            robot_pose.pose.position.y = (cy_goal - 50) * 0.05
            robot_pose.pose.orientation.w = 1.0

            return goal_pose, robot_pose
        return None, None

    def publish_static_transform(self):
        # Publish a static transform (e.g., from "map" to "odom")
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'
        transform_stamped.child_frame_id = 'odom'
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0  # No rotation

        self.tf_broadcaster.sendTransform(transform_stamped)
        self.get_logger().info('Static transform published')

def main(args=None):
    rclpy.init(args=args)
    node = MapProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
