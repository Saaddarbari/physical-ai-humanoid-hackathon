---
sidebar_position: 2
---

# Object Manipulation and Grasping

This chapter covers the implementation of object manipulation and grasping systems for humanoid robots, including perception, planning, and control for physical interaction with objects.

## Learning Goals

- Implement object detection and localization for manipulation
- Plan grasping and manipulation trajectories
- Control robot arms and hands for object manipulation
- Integrate perception and action for successful manipulation

## Core Concepts

- **Object Detection**: Identifying and localizing objects in the environment
- **Grasp Planning**: Determining how to grasp objects securely
- **Trajectory Planning**: Planning motion paths for manipulation
- **Force Control**: Managing contact forces during manipulation

## Implementation Section

### Object Detection for Manipulation

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_ros import TransformException
import math

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

        # Create publisher for object detections
        self.object_pub = self.create_publisher(MarkerArray, 'detected_objects', 10)
        self.object_pose_pub = self.create_publisher(PoseStamped, 'object_pose', 10)

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Object detection node started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to HSV for color-based object detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define range for red objects (adjust as needed for your objects)
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])

            # Create masks for red color
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2

            # Apply morphological operations to clean up the mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Create marker array for detected objects
            marker_array = MarkerArray()
            object_count = 0

            for contour in contours:
                # Filter by area to avoid small noise
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate center of object in image coordinates
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Draw bounding box and center on image
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(cv_image, (center_x, center_y), 5, (255, 0, 0), -1)

                    # Create marker for visualization
                    marker = Marker()
                    marker.header.frame_id = "camera_link"  # Camera frame
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "objects"
                    marker.id = object_count
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD

                    # Convert image coordinates to 3D world coordinates (simplified)
                    # This is a simplified approach - in reality you'd use depth info or stereo
                    marker.pose.position.x = (center_x - 320) * 0.002  # Approximate conversion
                    marker.pose.position.y = (center_y - 240) * 0.002  # Approximate conversion
                    marker.pose.position.z = 1.0  # Assumed distance
                    marker.pose.orientation.w = 1.0

                    # Scale based on object size
                    marker.scale.x = w * 0.002
                    marker.scale.y = h * 0.002
                    marker.scale.z = 0.1

                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8

                    marker_array.markers.append(marker)

                    # Publish object pose (simplified)
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = "camera_link"
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.pose.position.x = marker.pose.position.x
                    pose_msg.pose.position.y = marker.pose.position.y
                    pose_msg.pose.position.z = marker.pose.position.z
                    pose_msg.pose.orientation.w = 1.0

                    self.object_pose_pub.publish(pose_msg)

                    object_count += 1

            # Publish detection markers
            self.object_pub.publish(marker_array)

            # Publish annotated image (for debugging)
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            # We won't publish this in the main implementation to avoid circular dependencies

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

### Grasp Planning Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64MultiArray
from tf2_ros import TransformException
import numpy as np
import math

class GraspPlannerNode(Node):
    def __init__(self):
        super().__init__('grasp_planner_node')

        # Create publisher for grasp poses
        self.grasp_pose_pub = self.create_publisher(Pose, 'grasp_pose', 10)
        self.grasp_visualization_pub = self.create_publisher(MarkerArray, 'grasp_visualization', 10)

        # Create subscription for object poses
        self.object_pose_subscription = self.create_subscription(
            Pose,
            '/detected_object_pose',  # This would come from object detection
            self.object_pose_callback,
            10
        )

        # Robot hand parameters
        self.hand_width = 0.08  # Width of robot hand/gripper
        self.hand_depth = 0.05  # Depth of grasp
        self.hand_height = 0.03  # Height of grasp point

        self.get_logger().info('Grasp planner node started')

    def object_pose_callback(self, msg):
        """Plan grasp for detected object"""
        object_pos = msg.position
        object_orientation = msg.orientation

        # Plan grasp poses around the object
        grasp_poses = self.plan_grasps(object_pos)

        if grasp_poses:
            # For now, publish the first grasp pose (in a real system, you'd evaluate all and pick the best)
            best_grasp = grasp_poses[0]
            self.grasp_pose_pub.publish(best_grasp)

            # Visualize grasp poses
            self.visualize_grasps(grasp_poses, object_pos)

            self.get_logger().info(f'Planned {len(grasp_poses)} grasp poses for object at ({object_pos.x:.2f}, {object_pos.y:.2f}, {object_pos.z:.2f})')

    def plan_grasps(self, object_pos):
        """Plan multiple grasp poses around an object"""
        grasp_poses = []

        # Plan grasps from different directions around the object
        grasp_distance = 0.15  # Distance from object center for grasp approach
        grasp_height_offset = 0.05  # Height offset for top grasps

        # Side grasps (4 directions around the object)
        for angle in [0, math.pi/2, math.pi, 3*math.pi/2]:
            grasp_pose = Pose()

            # Calculate grasp position
            grasp_pose.position.x = object_pos.x + grasp_distance * math.cos(angle)
            grasp_pose.position.y = object_pos.y + grasp_distance * math.sin(angle)
            grasp_pose.position.z = object_pos.z  # Same height as object

            # Set orientation to face the object
            # Calculate quaternion for facing the object
            target_vector = np.array([object_pos.x - grasp_pose.position.x,
                                    object_pos.y - grasp_pose.position.y,
                                    0.0])
            target_vector = target_vector / np.linalg.norm(target_vector)

            # Create orientation that faces toward the object
            # This is a simplified approach - in reality you'd use proper quaternion math
            grasp_pose.orientation.z = math.sin(angle/2)
            grasp_pose.orientation.w = math.cos(angle/2)

            grasp_poses.append(grasp_pose)

        # Top grasp
        top_grasp = Pose()
        top_grasp.position.x = object_pos.x
        top_grasp.position.y = object_pos.y
        top_grasp.position.z = object_pos.z + grasp_height_offset

        # Top grasp orientation (typically with hand pointing down)
        top_grasp.orientation.x = 0.707  # 45-degree rotation around X axis
        top_grasp.orientation.y = 0.0
        top_grasp.orientation.z = 0.0
        top_grasp.orientation.w = 0.707

        grasp_poses.append(top_grasp)

        return grasp_poses

    def visualize_grasps(self, grasp_poses, object_pos):
        """Visualize planned grasps"""
        marker_array = MarkerArray()

        # Visualize the object (as a cube)
        object_marker = Marker()
        object_marker.header.frame_id = "map"
        object_marker.header.stamp = self.get_clock().now().to_msg()
        object_marker.ns = "object"
        object_marker.id = 0
        object_marker.type = Marker.CUBE
        object_marker.action = Marker.ADD

        object_marker.pose.position = object_pos
        object_marker.pose.orientation.w = 1.0
        object_marker.scale.x = 0.1
        object_marker.scale.y = 0.1
        object_marker.scale.z = 0.1
        object_marker.color.r = 0.0
        object_marker.color.g = 0.0
        object_marker.color.b = 1.0
        object_marker.color.a = 0.5

        marker_array.markers.append(object_marker)

        # Visualize each grasp pose
        for i, grasp_pose in enumerate(grasp_poses):
            # Grasp approach direction indicator
            approach_marker = Marker()
            approach_marker.header.frame_id = "map"
            approach_marker.header.stamp = self.get_clock().now().to_msg()
            approach_marker.ns = "grasp_approach"
            approach_marker.id = i + 1
            approach_marker.type = Marker.ARROW
            approach_marker.action = Marker.ADD

            # Set start and end points for the arrow
            # Start at grasp position
            approach_marker.points = []
            start_point = Point()
            start_point.x = grasp_pose.position.x
            start_point.y = grasp_pose.position.y
            start_point.z = grasp_pose.position.z
            approach_marker.points.append(start_point)

            # End point in the direction of approach (toward object)
            end_point = Point()
            # Simplified approach direction - in reality use orientation
            end_point.x = object_pos.x
            end_point.y = object_pos.y
            end_point.z = object_pos.z
            approach_marker.points.append(end_point)

            approach_marker.scale.x = 0.01  # Shaft diameter
            approach_marker.scale.y = 0.02  # Head diameter
            approach_marker.color.r = 1.0
            approach_marker.color.g = 0.0
            approach_marker.color.b = 0.0
            approach_marker.color.a = 1.0

            marker_array.markers.append(approach_marker)

            # Grasp position marker
            pos_marker = Marker()
            pos_marker.header.frame_id = "map"
            pos_marker.header.stamp = self.get_clock().now().to_msg()
            pos_marker.ns = "grasp_position"
            pos_marker.id = i + len(grasp_poses) + 1
            pos_marker.type = Marker.SPHERE
            pos_marker.action = Marker.ADD

            pos_marker.pose = grasp_pose
            pos_marker.scale.x = 0.02
            pos_marker.scale.y = 0.02
            pos_marker.scale.z = 0.02
            pos_marker.color.r = 0.0
            pos_marker.color.g = 1.0
            pos_marker.color.b = 0.0
            pos_marker.color.a = 0.8

            marker_array.markers.append(pos_marker)

        self.grasp_visualization_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = GraspPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down grasp planner node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [Object Detection in Robotics](https://arxiv.org/abs/1804.08182)
- [Robotic Grasping Techniques](https://ieeexplore.ieee.org/document/8206289)
- [Grasp Planning Algorithms](https://www.cs.cmu.edu/~./i3d/papers/iros09.pdf)
- [Manipulation in ROS](https://github.com/ros-planning/moveit)

## Quiz Questions

1. What are the key components of a robotic manipulation system?
2. How does grasp planning account for object properties and robot constraints?
3. What challenges arise when integrating perception and manipulation?