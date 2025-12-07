---
sidebar_position: 4
---

# NVIDIA Isaac Examples: Perception and Navigation

This chapter provides comprehensive NVIDIA Isaac code examples covering perception systems, navigation algorithms, and AI integration for robotics applications.

## Learning Goals

- Implement perception systems using NVIDIA Isaac tools
- Apply navigation and path planning algorithms
- Integrate AI models for object detection and recognition
- Use Isaac Sim for photorealistic simulation and synthetic data

## Core Concepts

- **Perception Systems**: Processing sensor data to understand the environment
- **Navigation and Path Planning**: Algorithms for finding optimal routes
- **Sim-to-Real Transfer**: Techniques for applying simulation-trained models to real robots
- **Synthetic Data Generation**: Creating training data using simulation

## Implementation Section

### Isaac ROS Perception Pipeline

```python
# perception_pipeline.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
from typing import List, Tuple

class IsaacPerceptionNode(Node):
    def __init__(self):
        super().__init__('isaac_perception_node')

        # Create subscriptions
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publishers
        self.detection_publisher = self.create_publisher(Detection2DArray, '/detections', 10)
        self.visualization_publisher = self.create_publisher(Image, '/perception_visualization', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Camera parameters (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Initialize object detection model
        self.initialize_detection_model()

        self.get_logger().info('Isaac perception node started')

    def initialize_detection_model(self):
        """Initialize object detection model (using a pre-trained model)"""
        try:
            # Using a pre-trained model - in a real implementation, you would load
            # a model trained specifically for your use case
            self.detection_model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.detection_model.eval()
            self.get_logger().info('Detection model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load detection model: {e}')
            # Fallback to OpenCV-based detection
            self.detection_model = None

    def camera_info_callback(self, msg):
        """Update camera parameters from camera info"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        """Process camera image for object detection"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform object detection
            if self.detection_model is not None:
                detections = self.detect_objects_yolo(cv_image)
            else:
                # Fallback: simple color-based detection
                detections = self.detect_objects_cv(cv_image)

            # Create detection message
            detection_msg = self.create_detection_message(detections, msg.header)

            # Publish detections
            self.detection_publisher.publish(detection_msg)

            # Create visualization
            vis_image = self.visualize_detections(cv_image, detections)
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            vis_msg.header = msg.header
            self.visualization_publisher.publish(vis_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_objects_yolo(self, image):
        """Detect objects using YOLO model"""
        try:
            # Perform inference
            results = self.detection_model(image)

            # Extract detections
            detections = []
            for *xyxy, conf, cls in results.xyxy[0].tolist():
                if conf > 0.5:  # Confidence threshold
                    x1, y1, x2, y2 = map(int, xyxy)
                    detections.append({
                        'class_id': int(cls),
                        'confidence': conf,
                        'bbox': (x1, y1, x2-x1, y2-y1),  # x, y, width, height
                        'center': ((x1+x2)//2, (y1+y2)//2)
                    })

            return detections
        except Exception as e:
            self.get_logger().error(f'YOLO detection error: {e}')
            return []

    def detect_objects_cv(self, image):
        """Fallback detection using OpenCV (color-based)"""
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red objects
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Apply morphological operations
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                detections.append({
                    'class_id': 0,  # Red object
                    'confidence': 0.7,  # Estimated confidence
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })

        return detections

    def create_detection_message(self, detections, header):
        """Create detection message from detection results"""
        detection_array = Detection2DArray()
        detection_array.header = header

        for det in detections:
            detection = Detection2D()
            detection.header = header

            # Bounding box
            detection.bbox.size_x = det['bbox'][2]
            detection.bbox.size_y = det['bbox'][3]
            detection.bbox.center.x = det['center'][0]
            detection.bbox.center.y = det['center'][1]

            # Hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = det['confidence']
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        return detection_array

    def visualize_detections(self, image, detections):
        """Visualize detections on image"""
        vis_image = image.copy()

        for det in detections:
            x, y, w, h = det['bbox']
            center_x, center_y = det['center']

            # Draw bounding box
            cv2.rectangle(vis_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Draw center
            cv2.circle(vis_image, (center_x, center_y), 5, (255, 0, 0), -1)

            # Add label
            label = f"Object {det['class_id']}: {det['confidence']:.2f}"
            cv2.putText(vis_image, label, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return vis_image

def main(args=None):
    rclpy.init(args=args)
    node = IsaacPerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Isaac perception node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Isaac Navigation and Path Planning

```python
# navigation_planner.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np
from collections import deque
import math

class IsaacNavigationPlanner(Node):
    def __init__(self):
        super().__init__('isaac_navigation_planner')

        # Create subscriptions
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        # Create publishers
        self.path_publisher = self.create_publisher(Path, '/plan', 10)
        self.global_plan_publisher = self.create_publisher(Path, '/global_plan', 10)

        # Map data
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None

        # Robot position (for simulation - in real system, use odometry)
        self.current_x = 0.0
        self.current_y = 0.0

        # Goal position
        self.goal_x = None
        self.goal_y = None

        # Initialize A* path planner
        self.path_planner = AStarPlanner()

        self.get_logger().info('Isaac navigation planner started')

    def map_callback(self, msg):
        """Process occupancy grid map"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

    def scan_callback(self, msg):
        """Process laser scan for local obstacle avoidance"""
        # In a real system, this would update a local costmap
        # For this example, we'll just store the scan data
        self.laser_scan = msg

    def goal_callback(self, msg):
        """Process navigation goal"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        # Plan path to goal
        self.plan_path()

    def plan_path(self):
        """Plan path from current position to goal"""
        if self.map_data is None or self.goal_x is None or self.goal_y is None:
            return

        # Convert world coordinates to map coordinates
        start_map_x, start_map_y = self.world_to_map(self.current_x, self.current_y)
        goal_map_x, goal_map_y = self.world_to_map(self.goal_x, self.goal_y)

        if start_map_x is None or start_map_y is None or goal_map_x is None or goal_map_y is None:
            self.get_logger().warn('Start or goal position is outside map bounds')
            return

        # Plan path using A*
        path = self.path_planner.plan_path(
            self.map_data,
            (start_map_x, start_map_y),
            (goal_map_x, goal_map_y)
        )

        if path:
            # Convert path to ROS Path message
            ros_path = self.create_path_message(path)
            self.path_publisher.publish(ros_path)
            self.global_plan_publisher.publish(ros_path)
            self.get_logger().info(f'Planned path with {len(path)} waypoints')
        else:
            self.get_logger().warn('No path found to goal')

    def world_to_map(self, x, y):
        """Convert world coordinates to map indices"""
        if self.map_origin is None:
            return None, None

        map_x = int((x - self.map_origin.position.x) / self.map_resolution)
        map_y = int((y - self.map_origin.position.y) / self.map_resolution)

        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            return map_x, map_y
        else:
            return None, None

    def map_to_world(self, map_x, map_y):
        """Convert map indices to world coordinates"""
        if self.map_origin is None:
            return None, None

        world_x = map_x * self.map_resolution + self.map_origin.position.x
        world_y = map_y * self.map_resolution + self.map_origin.position.y

        return world_x, world_y

    def create_path_message(self, path):
        """Create ROS Path message from path coordinates"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for map_x, map_y in path:
            world_x, world_y = self.map_to_world(map_x, map_y)
            if world_x is not None and world_y is not None:
                pose = PoseStamped()
                pose.header.stamp = path_msg.header.stamp
                pose.header.frame_id = 'map'
                pose.pose.position.x = world_x
                pose.pose.position.y = world_y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0

                path_msg.poses.append(pose)

        return path_msg

class AStarPlanner:
    """A* path planning implementation"""

    def __init__(self):
        self.motion = [
            [-1, 0],    # go up
            [0, -1],    # go left
            [1, 0],     # go down
            [0, 1],     # go right
            [-1, -1],   # go up-left (diagonal)
            [-1, 1],    # go up-right (diagonal)
            [1, -1],    # go down-left (diagonal)
            [1, 1]      # go down-right (diagonal)
        ]

    def plan_path(self, occupancy_grid, start, goal):
        """
        Plan path using A* algorithm

        Args:
            occupancy_grid: 2D numpy array with occupancy values (0=free, 100=occupied)
            start: (x, y) tuple for start position in map coordinates
            goal: (x, y) tuple for goal position in map coordinates

        Returns:
            List of (x, y) tuples representing the path, or None if no path found
        """
        start_node = self.Node(start[0], start[1], 0.0, -1)
        goal_node = self.Node(goal[0], goal[1], 0.0, -1)

        # Create open and closed sets
        open_set = {self.calculate_index(start_node, occupancy_grid.shape[1]): start_node}
        closed_set = {}

        while open_set:
            # Find node with minimum cost
            current_id = min(open_set, key=lambda x: open_set[x].cost + self.heuristic(open_set[x], goal_node))
            current_node = open_set[current_id]

            # Check if we reached the goal
            if self.verify_final_node(current_node, goal_node):
                goal_node.parent_index = current_node.parent_index
                goal_node.cost = current_node.cost
                break

            # Remove current node from open set and add to closed set
            del open_set[current_id]
            closed_set[current_id] = current_node

            # Explore neighbors
            for move_x, move_y in self.motion:
                node = self.Node(
                    current_node.x + move_x,
                    current_node.y + move_y,
                    current_node.cost,
                    current_id
                )

                # Check if node is valid
                n_id = self.calculate_index(node, occupancy_grid.shape[1])

                if n_id in closed_set:
                    continue

                if not self.verify_node(node, occupancy_grid):
                    continue

                # Update cost
                node.cost += math.sqrt(move_x**2 + move_y**2)
                node.cost += self.calc_obstacle_cost(node, occupancy_grid)

                if n_id in open_set:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node
                else:
                    open_set[n_id] = node

        # Generate final course
        if len(closed_set) == 0:
            self.get_logger().warn('A* failed to find a path!')
            return None

        path = [[goal_node.x, goal_node.y]]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            path.append([n.x, n.y])
            parent_index = n.parent_index

        return path[::-1]  # Reverse to get path from start to goal

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def calculate_index(self, node, max_x):
        return node.y * max_x + node.x

    def verify_final_node(self, node, goal_node):
        d = self.heuristic(node, goal_node)
        if d <= 1.0:  # Consider as reached if within 1 grid cell
            return True
        return False

    def heuristic(self, n1, n2):
        """Calculate heuristic distance between two nodes"""
        return math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

    def verify_node(self, node, occupancy_grid):
        """Verify if node is valid (not out of bounds or occupied)"""
        if node.x < 0 or node.y < 0:
            return False

        if node.x >= occupancy_grid.shape[1] or node.y >= occupancy_grid.shape[0]:
            return False

        if occupancy_grid[node.y][node.x] >= 50:  # Occupied or unknown
            return False

        return True

    def calc_obstacle_cost(self, node, occupancy_grid):
        """Calculate additional cost for obstacles"""
        # Simple obstacle cost - in reality, this would consider more factors
        if 0 <= node.y < occupancy_grid.shape[0] and 0 <= node.x < occupancy_grid.shape[1]:
            if occupancy_grid[node.y][node.x] > 0:
                return occupancy_grid[node.y][node.x] / 100.0  # Normalize to 0-1 range
        return 0.0

    def get_logger(self):
        """Mock logger for the planner class"""
        import logging
        return logging.getLogger(__name__)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacNavigationPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Isaac navigation planner')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Isaac Sim Integration Example

```python
# sim_integration.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import random

class IsaacSimIntegrationNode(Node):
    def __init__(self):
        super().__init__('isaac_sim_integration_node')

        # Create publishers for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriptions for simulated sensors
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Float32MultiArray,
            '/isaac_sim/robot_state',
            self.odom_callback,
            10
        )

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Robot state
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0
        self.laser_data = []

        # Create timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Isaac Sim integration node started')

    def camera_callback(self, msg):
        """Process camera image from Isaac Sim"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # In Isaac Sim, you might have additional semantic segmentation or depth data
            # For this example, we'll just process the RGB image
            self.process_camera_data(cv_image)

        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')

    def laser_callback(self, msg):
        """Process laser scan from Isaac Sim"""
        self.laser_data = list(msg.ranges)

    def odom_callback(self, msg):
        """Process robot state from Isaac Sim"""
        if len(msg.data) >= 3:
            self.position_x = msg.data[0]
            self.position_y = msg.data[1]
            self.orientation = msg.data[2]

    def process_camera_data(self, image):
        """Process camera data for perception tasks"""
        # In a real Isaac Sim integration, this might connect to
        # Isaac Sim's perception pipeline
        height, width = image.shape[:2]

        # Example: detect objects using simple color-based detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red objects
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Process detected objects
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small noise
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Calculate relative position in image (for Isaac Sim object localization)
                rel_x = (center_x - width/2) / (width/2)  # -1 to 1
                rel_y = (center_y - height/2) / (height/2)  # -1 to 1

                self.get_logger().info(f'Detected object at relative position: ({rel_x:.2f}, {rel_y:.2f})')

    def control_loop(self):
        """Main control loop for robot navigation"""
        cmd = Twist()

        # Simple obstacle avoidance using laser data
        if self.laser_data:
            # Check for obstacles in front (simplified)
            front_ranges = self.laser_data[160:200]  # Front 40 degrees worth of readings
            min_front_dist = min([r for r in front_ranges if not np.isnan(r)], default=float('inf'))

            if min_front_dist < 1.0:  # Obstacle detected
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn right
            else:
                cmd.linear.x = 0.3  # Move forward
                cmd.angular.z = 0.0  # No turn

            # Publish command
            self.cmd_vel_publisher.publish(cmd)

    def navigate_to_waypoint(self, target_x, target_y, tolerance=0.1):
        """Navigate to a specific waypoint in Isaac Sim"""
        cmd = Twist()

        # Calculate distance to target
        distance = np.sqrt((target_x - self.position_x)**2 + (target_y - self.position_y)**2)

        if distance > tolerance:
            # Calculate angle to target
            target_angle = np.arctan2(target_y - self.position_y, target_x - self.position_x)
            angle_diff = target_angle - self.orientation

            # Normalize angle difference
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi

            # Proportional control for angular velocity
            cmd.angular.z = np.clip(angle_diff * 1.0, -1.0, 1.0)

            # Move forward if roughly aligned with target
            if abs(angle_diff) < 0.2:
                cmd.linear.x = min(0.3, distance)  # Scale speed with distance

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = IsaacSimIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Isaac Sim integration node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Domain Randomization for Sim-to-Real Transfer

```python
# domain_randomization.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import random

class DomainRandomizationNode(Node):
    def __init__(self):
        super().__init__('domain_randomization_node')

        # Create subscription for camera images
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        # Create publisher for randomized images
        self.randomized_publisher = self.create_publisher(Image, '/randomized_image', 10)

        # Create publisher for domain parameters
        self.params_publisher = self.create_publisher(Float32MultiArray, '/domain_params', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Domain randomization parameters
        self.randomization_params = {
            'brightness_range': (-50, 50),
            'contrast_range': (0.8, 1.2),
            'saturation_range': (0.8, 1.2),
            'hue_range': (-10, 10),
            'blur_range': (0, 3),
            'noise_range': (0, 0.05),
            'lighting_angle_range': (0, 360),
            'lighting_intensity_range': (0.5, 1.5)
        }

        # Create timer for randomization
        self.randomization_timer = self.create_timer(0.1, self.randomize_parameters)

        # Current randomization values
        self.current_params = self.generate_random_params()

        self.get_logger().info('Domain randomization node started')

    def generate_random_params(self):
        """Generate random domain parameters"""
        params = {}
        for param, (min_val, max_val) in self.randomization_params.items():
            if 'range' in param:
                params[param] = random.uniform(min_val, max_val)
            else:
                params[param] = random.uniform(min_val, max_val)
        return params

    def randomize_parameters(self):
        """Generate new random parameters"""
        self.current_params = self.generate_random_params()

        # Publish current parameters
        params_msg = Float32MultiArray()
        params_msg.data = [
            self.current_params['brightness_range'],
            self.current_params['contrast_range'],
            self.current_params['saturation_range'],
            self.current_params['hue_range'],
            self.current_params['blur_range'],
            self.current_params['noise_range']
        ]
        self.params_publisher.publish(params_msg)

        self.get_logger().info(f'New domain parameters: brightness={self.current_params["brightness_range"]:.2f}, '
                              f'contrast={self.current_params["contrast_range"]:.2f}, '
                              f'saturation={self.current_params["saturation_range"]:.2f}')

    def camera_callback(self, msg):
        """Process camera image and apply domain randomization"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Apply domain randomization
            randomized_image = self.apply_domain_randomization(cv_image)

            # Publish randomized image
            randomized_msg = self.bridge.cv2_to_imgmsg(randomized_image, encoding='bgr8')
            randomized_msg.header = msg.header
            self.randomized_publisher.publish(randomized_msg)

        except Exception as e:
            self.get_logger().error(f'Error applying domain randomization: {e}')

    def apply_domain_randomization(self, image):
        """Apply domain randomization techniques to image"""
        # Convert to HSV for some operations
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float64)

        # Adjust brightness
        brightness_factor = self.current_params['brightness_range']
        hsv[:, :, 2] = np.clip(hsv[:, :, 2] + brightness_factor, 0, 255)

        # Adjust saturation
        saturation_factor = self.current_params['saturation_range']
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation_factor, 0, 255)

        # Convert back to BGR
        randomized_image = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)

        # Apply contrast
        contrast_factor = self.current_params['contrast_range']
        randomized_image = randomized_image.astype(np.float64)
        randomized_image = np.clip(128 + contrast_factor * (randomized_image - 128), 0, 255)
        randomized_image = randomized_image.astype(np.uint8)

        # Apply Gaussian blur
        blur_factor = int(self.current_params['blur_range'])
        if blur_factor > 0:
            kernel_size = blur_factor * 2 + 1  # Must be odd
            randomized_image = cv2.GaussianBlur(randomized_image, (kernel_size, kernel_size), 0)

        # Add noise
        noise_factor = self.current_params['noise_range']
        if noise_factor > 0:
            noise = np.random.normal(0, noise_factor * 255, randomized_image.shape)
            randomized_image = np.clip(randomized_image + noise, 0, 255).astype(np.uint8)

        return randomized_image

def main(args=None):
    rclpy.init(args=args)
    node = DomainRandomizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down domain randomization node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [NVIDIA Isaac ROS Documentation](https://docs.nvidia.com/isaac/isaac_ros/)
- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [ROS 2 Navigation](https://navigation.ros.org/)
- [Computer Vision in Robotics](https://docs.ros.org/en/humble/p/ vision_opencv/)

## Quiz Questions

1. What are the key components of a perception system in NVIDIA Isaac?
2. How does A* path planning work in robotics navigation?
3. What is domain randomization and why is it important for sim-to-real transfer?