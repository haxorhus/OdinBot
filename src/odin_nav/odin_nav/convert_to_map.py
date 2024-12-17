#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import os

class ImageProcessorNode(Node):
    def __init__(self):
        super().__init__('image_processor_node')

        # Publishers for the positions
        self.goal_pub = self.create_publisher(PoseStamped, '/odin/goal', 10)
        self.robot_position_pub = self.create_publisher(PoseStamped, '/odin/robot_position', 10)
        self.yellow_position_pub = self.create_publisher(PoseStamped, '/odin/box', 10)

        # Timer to process the image every 0.5 seconds
        self.timer = self.create_timer(0.5, self.process_image)

        # CvBridge for image processing
        self.bridge = CvBridge()

        # Path to input image and output map files
        self.image_path = '/home/jose/microros_ws/src/img/cropped_image.png'
        self.map_dir = '/home/jose/microros_ws/src/maps'

        # Configuration variables
        self.map_resolution = 0.005  # Resolution in meters/pixel
        self.map_size_meters = 2.7   # Map size in meters

        # Variable to store the initial robot position
        self.initial_robot_position = None

        # Informational message
        self.get_logger().info('Nodo de procesamiento de imágenes iniciado')

    def process_image(self):
        # Read the image from the file
        image = cv2.imread(self.image_path)
        if image is None:
            self.get_logger().error(f"Error al cargar la imagen desde {self.image_path}")
            return

        # Convert the cropped image to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Get image dimensions
        h, w = image.shape[:2]

        # Calculate and publish positions of the robot and the goal
        goal_pose, robot_pose,box_pose, marked_image = self.calculate_positions(image, w, h)

        # Save the binary map as PGM
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask_red1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2

        binary_red = cv2.bitwise_not(mask_red)  # Invert for white background
        binary_red = cv2.flip(binary_red, 0)  # Vertical flip

        pgm_filename = os.path.join(self.map_dir, 'map.pgm')
        cv2.imwrite(pgm_filename, binary_red)

        # Save the YAML file with the origin set to the robot's transformed position
        yaml_filename = os.path.join(self.map_dir, 'map.yaml')
        with open(yaml_filename, 'w') as yaml_file:
            yaml_file.write(f"image: map.pgm\n")
            yaml_file.write(f"resolution: {self.map_resolution}\n")
            yaml_file.write(f"origin: [0.0, 0.0, 0.0]\n")
            yaml_file.write("negate: 0\n")
            yaml_file.write("occupied_thresh: 0.65\n")
            yaml_file.write("free_thresh: 0.196\n")

        # Publish robot position
        self.robot_position_pub.publish(robot_pose)
        self.goal_pub.publish(goal_pose)
        self.yellow_position_pub.publish(box_pose)

        # Show the image with highlighted contours
        cv2.waitKey(1)

    def calculate_positions(self, cropped_image, width_px, height_px):
        hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
        marked_image = cropped_image.copy()

        # Robot detection
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_green:
            max_contour_green = max(contours_green, key=cv2.contourArea)
            M_green = cv2.moments(max_contour_green)
            if M_green['m00'] != 0:
                cx_goal = int(M_green['m10'] / M_green['m00'])
                cy_goal = int(M_green['m01'] / M_green['m00'])
                cv2.drawContours(marked_image, [max_contour_green], -1, (0, 0, 255), 2)  # Red contour
            else:
                cx_goal, cy_goal = 0, 0
        else:
            cx_goal, cy_goal = 0, 0

        # --- ROBOT DETECTION ---
        gray_cropped = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        gray_cropped = cv2.medianBlur(gray_cropped, 5)
        circles = cv2.HoughCircles(
            gray_cropped, 
            cv2.HOUGH_GRADIENT, 
            dp=1.2, 
            minDist=30, 
            param1=150, 
            param2=40, 
            minRadius=10, 
            maxRadius=30
        )

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            cx_robot, cy_robot, radius = circles[0]
            cv2.circle(marked_image, (cx_robot, cy_robot), radius, (255, 0, 0), 2)  # Blue contour
        else:
            cx_robot, cy_robot = 0, 0
        
        # --- YELLOW OBJECT DETECTION ---
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_yellow:
            max_contour_yellow = max(contours_yellow, key=cv2.contourArea)
            M_yellow = cv2.moments(max_contour_yellow)
            if M_yellow['m00'] != 0:
                cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])
                cy_yellow = int(M_yellow['m01'] / M_yellow['m00'])
                cv2.drawContours(marked_image, [max_contour_yellow], -1, (0, 255, 255), 2)  # Yellow contour
            else:
                cx_yellow, cy_yellow = 0, 0
        else:
            cx_yellow, cy_yellow = 0, 0


        # Conversion from pixels to meters
        scale_factor_x = self.map_size_meters / width_px
        scale_factor_y = self.map_size_meters / height_px
        
        # Create PoseStamped for the goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = cx_goal * scale_factor_x
        goal_pose.pose.position.y = (height_px - cy_goal) * scale_factor_y
        goal_pose.pose.orientation.w = 1.0
        
        # Create PoseStamped for robot position
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = 'odom'
        robot_pose.header.stamp = self.get_clock().now().to_msg()
        robot_pose.pose.position.x = cx_robot * scale_factor_x
        robot_pose.pose.position.y = (height_px - cy_robot) * scale_factor_y
        robot_pose.pose.orientation.w = 1.0

        # Create PoseStamped for yellow position
        yellow_pose = PoseStamped()
        yellow_pose.header.frame_id = 'map'
        yellow_pose.header.stamp = self.get_clock().now().to_msg()
        yellow_pose.pose.position.x = cx_yellow * scale_factor_x
        yellow_pose.pose.position.y = (height_px - cy_yellow) * scale_factor_y
        yellow_pose.pose.orientation.w = 1.0

        return goal_pose, robot_pose,yellow_pose, marked_image


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
