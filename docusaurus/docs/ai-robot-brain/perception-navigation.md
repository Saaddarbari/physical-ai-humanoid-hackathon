---
sidebar_position: 1
---

# Perception and Navigation Systems

This chapter covers AI-based perception systems for environment understanding and navigation algorithms for autonomous robot movement.

## Learning Goals

- Implement computer vision systems for robot perception
- Apply navigation algorithms for path planning and obstacle avoidance
- Integrate sensor data for environment mapping
- Use machine learning for object detection and recognition

## Core Concepts

- **Perception Systems**: Processing sensor data to understand the environment
- **SLAM (Simultaneous Localization and Mapping)**: Building maps while localizing
- **Path Planning**: Algorithms for finding optimal routes
- **Obstacle Avoidance**: Techniques for navigating around obstacles

## Implementation Section

### Basic Object Detection Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import cv2
from cv_bridge import CvBridge
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Create subscription to camera image
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create publisher for detection results
        self.detection_pub = self.create_publisher(MarkerArray, 'object_detections', 10)

        # Create publisher for annotated image
        self.annotated_image_pub = self.create_publisher(Image, 'annotated_image', 10)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        self.get_logger().info('Object detection node started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform simple color-based object detection (red objects)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define range for red color
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])

            # Create masks for red color
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create marker array for detected objects
            marker_array = MarkerArray()
            detection_count = 0

            for contour in contours:
                # Filter by area to avoid small noise
                if cv2.contourArea(contour) > 500:
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Draw bounding box on image
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'Red Object {detection_count}',
                               (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Create marker for visualization
                    marker = Marker()
                    marker.header.frame_id = "camera_link"  # Assuming camera frame
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "object_detections"
                    marker.id = detection_count
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD

                    # Convert image coordinates to 3D (simplified - assumes known depth)
                    # In real application, you'd use depth data
                    marker.pose.position.x = (x + w/2) / 320.0  # Normalize to camera FOV
                    marker.pose.position.y = (y + h/2) / 240.0  # Normalize to camera FOV
                    marker.pose.position.z = 1.0  # Assumed distance
                    marker.pose.orientation.w = 1.0

                    # Scale based on object size
                    marker.scale.x = w / 320.0
                    marker.scale.y = h / 240.0
                    marker.scale.z = 0.1

                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8

                    marker_array.markers.append(marker)
                    detection_count += 1

            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)

            # Publish detection markers
            self.detection_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object detection node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Simple Navigation Node with Obstacle Avoidance

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Create subscription to laser scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create subscription to odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Navigation parameters
        self.target_x = 5.0  # Target position
        self.target_y = 5.0
        self.safe_distance = 0.5  # Minimum distance to obstacles
        self.linear_speed = 0.3
        self.angular_speed = 0.5

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laser_ranges = []

        self.get_logger().info('Navigation node started')

    def odom_callback(self, msg):
        # Update current position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))

    def scan_callback(self, msg):
        self.laser_ranges = list(msg.ranges)

        # Calculate distance to target
        target_distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)

        # Check if close enough to target
        if target_distance < 0.3:
            self.stop_robot()
            self.get_logger().info('Reached target position!')
            return

        # Calculate angle to target
        target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
        angle_diff = target_angle - self.current_yaw

        # Normalize angle difference
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # Create twist message
        cmd_vel = Twist()

        # Check for obstacles in front
        min_front_distance = float('inf')
        front_scan_start = len(self.laser_ranges) // 2 - 30
        front_scan_end = len(self.laser_ranges) // 2 + 30

        if front_scan_start < 0:
            front_scan_start = 0
        if front_scan_end >= len(self.laser_ranges):
            front_scan_end = len(self.laser_ranges) - 1

        for i in range(front_scan_start, front_scan_end):
            if i < len(self.laser_ranges) and not math.isnan(self.laser_ranges[i]):
                if self.laser_ranges[i] < min_front_distance:
                    min_front_distance = self.laser_ranges[i]

        # Obstacle avoidance logic
        if min_front_distance < self.safe_distance:
            # Obstacle detected, turn away
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = self.angular_speed
        else:
            # No obstacle, move toward target
            if abs(angle_diff) > 0.1:  # Need to turn
                cmd_vel.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                cmd_vel.linear.x = 0.0
            else:  # Move forward
                cmd_vel.linear.x = self.linear_speed
                cmd_vel.angular.z = 0.0

        # Publish command
        self.cmd_vel_publisher.publish(cmd_vel)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down navigation node')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [Navigation2 Documentation](https://navigation.ros.org/)
- [Computer Vision in ROS](https://wiki.ros.org/vision_opencv)
- [SLAM Tutorials](https://navigation.ros.org/tutorials/docs/navigation2_tutorials.html)
- [Object Detection with OpenCV](https://docs.opencv.org/)

## Quiz Questions

1. What is the difference between perception and navigation in robotics?
2. How does SLAM enable robots to operate in unknown environments?
3. What are the key components of a navigation stack?