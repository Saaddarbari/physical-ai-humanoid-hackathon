---
sidebar_position: 3
---

# Gazebo Simulation Examples: Physics and Environment

This chapter provides comprehensive Gazebo simulation code examples covering physics simulation, environment creation, and sensor integration for robotics development.

## Learning Goals

- Create realistic simulation environments with proper physics properties
- Integrate various sensors (LiDAR, cameras, IMUs) in simulation
- Control robots in simulated environments
- Validate robot behaviors in simulation before real-world deployment

## Core Concepts

- **Physics Simulation**: Accurate modeling of real-world physics for robot testing
- **Sensor Simulation**: Creating realistic sensor data for development and training
- **Environment Building**: Creating worlds with proper collision and visual properties
- **Simulation Parameters**: Configuring physics and rendering properties

## Implementation Section

### Basic Gazebo World with Robot

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_robot_world">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Simple differential drive robot -->
    <model name="simple_robot">
      <pose>0 0 0.1 0 0 0</pose>

      <!-- Robot body -->
      <link name="chassis">
        <pose>0 0 0.1 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.3 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.3 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.02083</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.05417</iyy>
            <iyz>0</iyz>
            <izz>0.075</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Left wheel -->
      <link name="left_wheel">
        <pose>-0.15 0.2 0 0 1.5707 1.5707</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.05</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.05</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.2</mass>
          <inertia>
            <ixx>0.0015</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0015</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Right wheel -->
      <link name="right_wheel">
        <pose>-0.15 -0.2 0 0 1.5707 1.5707</pose>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.05</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.05</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.2</mass>
          <inertia>
            <ixx>0.0015</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0015</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>

      <!-- Joints -->
      <joint name="left_wheel_joint" type="continuous">
        <parent>chassis</parent>
        <child>left_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>

      <joint name="right_wheel_joint" type="continuous">
        <parent>chassis</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>

      <!-- Simple controller plugin (for demonstration) -->
      <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.4</wheel_separation>
        <wheel_diameter>0.2</wheel_diameter>
        <max_wheel_torque>20</max_wheel_torque>
        <max_wheel_acceleration>1.0</max_wheel_acceleration>
        <command_topic>cmd_vel</command_topic>
        <odometry_topic>odom</odometry_topic>
        <odometry_frame>odom</odometry_frame>
        <robot_base_frame>chassis</robot_base_frame>
      </plugin>
    </model>

    <!-- Physics properties -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

### LiDAR Sensor Integration

```xml
  <!-- LiDAR Sensor -->
  <link name="lidar_link">
    <pose>0.2 0 0.15 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.05</length>
        </cylinder>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>0.05</radius>
          <length>0.05</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
        <diffuse>0.5 0.5 0.5 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>0.1</mass>
      <inertia>
        <ixx>1.5e-05</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>1.5e-05</iyy>
        <iyz>0</iyz>
        <izz>2.5e-05</izz>
      </inertia>
    </inertial>

    <!-- Gazebo LiDAR plugin -->
    <sensor name="laser_sensor" type="ray">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>laser</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent>chassis</parent>
    <child>lidar_link</child>
    <pose>0.2 0 0.15 0 0 0</pose>
  </joint>
```

### Camera Sensor Integration

```xml
  <!-- RGB-D Camera -->
  <link name="camera_link">
    <pose>0.25 0 0.1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.02 0.05 0.02</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.02 0.05 0.02</size>
        </box>
      </geometry>
      <material>
        <ambient>0 0 0 1</ambient>
        <diffuse>0 0 0 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>0.01</mass>
      <inertia>
        <ixx>8.33e-07</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>4.67e-07</iyy>
        <iyz>0</iyz>
        <izz>8.33e-07</izz>
      </inertia>
    </inertial>

    <!-- Gazebo camera plugin -->
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <camera name="head">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>camera</namespace>
          <remapping>~/image_raw:=image_raw</remapping>
          <remapping>~/camera_info:=camera_info</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent>chassis</parent>
    <child>camera_link</child>
    <pose>0.25 0 0.1 0 0 0</pose>
  </joint>
```

### IMU Sensor Integration

```xml
  <!-- IMU Sensor -->
  <link name="imu_link">
    <pose>0 0 0.1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>0.01 0.01 0.01</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.01 0.01 0.01</size>
        </box>
      </geometry>
      <material>
        <ambient>0 1 0 1</ambient>
        <diffuse>0 1 0 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>0.001</mass>
      <inertia>
        <ixx>8.33e-09</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>8.33e-09</iyy>
        <iyz>0</iyz>
        <izz>8.33e-09</izz>
      </inertia>
    </inertial>

    <!-- Gazebo IMU plugin -->
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>false</visualize>
      <topic>__default_topic__</topic>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>imu</namespace>
          <remapping>~/out:=data</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
      <pose>0 0 0 0 0 0</pose>
    </sensor>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent>chassis</parent>
    <child>imu_link</child>
    <pose>0 0 0.1 0 0 0</pose>
  </joint>
```

### Simulation Control Node

```python
# simulation_control.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class SimulationControlNode(Node):
    def __init__(self):
        super().__init__('simulation_control_node')

        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.velocity_publisher = self.create_publisher(Vector3, '/sim_velocity', 10)

        # Create subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laser_ranges = []
        self.imu_data = None

        self.get_logger().info('Simulation control node started')

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        self.current_yaw = np.arctan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )

    def scan_callback(self, msg):
        """Store laser scan data"""
        self.laser_ranges = list(msg.ranges)

    def imu_callback(self, msg):
        """Store IMU data"""
        self.imu_data = msg

    def image_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Simple processing: detect red objects
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 50, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 50, 50])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw bounding boxes for detected objects
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # Filter small noise
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Display the processed image
            cv2.imshow('Processed Camera View', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def control_loop(self):
        """Main control loop"""
        # Example: Simple obstacle avoidance
        cmd = Twist()

        if self.laser_ranges:
            # Check for obstacles in front
            front_ranges = self.laser_ranges[160:200]  # Front 40 degrees
            min_front_dist = min([r for r in front_ranges if not np.isnan(r)], default=float('inf'))

            if min_front_dist < 1.0:  # Obstacle detected
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn right
            else:
                cmd.linear.x = 0.3  # Move forward
                cmd.angular.z = 0.0

        # Publish command
        self.cmd_vel_publisher.publish(cmd)

        # Publish velocity for monitoring
        vel_msg = Vector3()
        vel_msg.x = cmd.linear.x
        vel_msg.z = cmd.angular.z
        self.velocity_publisher.publish(vel_msg)

    def move_to_position(self, target_x, target_y, tolerance=0.1):
        """Move robot to target position"""
        cmd = Twist()

        distance = np.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

        if distance > tolerance:
            # Calculate angle to target
            target_angle = np.arctan2(target_y - self.current_y, target_x - self.current_x)
            angle_diff = target_angle - self.current_yaw

            # Normalize angle
            while angle_diff > np.pi:
                angle_diff -= 2 * np.pi
            while angle_diff < -np.pi:
                angle_diff += 2 * np.pi

            # Turn towards target
            if abs(angle_diff) > 0.1:
                cmd.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Move forward
                cmd.linear.x = min(0.3, distance)  # Scale speed with distance

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimulationControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down simulation control node')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced World with Multiple Obstacles

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="advanced_world">
    <!-- Include standard models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Walls -->
    <model name="wall_1">
      <pose>5 0 0.5 0 0 0</pose>
      <link name="wall_1_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>833.3</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>4.2</iyy>
            <iyz>0</iyz>
            <izz>837.5</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_2">
      <pose>-5 0 0.5 0 0 0</pose>
      <link name="wall_2_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>833.3</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>4.2</iyy>
            <iyz>0</iyz>
            <izz>837.5</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_3">
      <pose>0 5 0.5 0 0 1.5707</pose>
      <link name="wall_3_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>833.3</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>4.2</iyy>
            <iyz>0</iyz>
            <izz>837.5</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="wall_4">
      <pose>0 -5 0.5 0 0 1.5707</pose>
      <link name="wall_4_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 10 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>833.3</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>4.2</iyy>
            <iyz>0</iyz>
            <izz>837.5</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Obstacles -->
    <model name="cylinder_obstacle">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="cylinder_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1.25</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1.25</iyy>
            <iyz>0</iyz>
            <izz>0.45</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="box_obstacle">
      <pose>-2 -2 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.8 0.8 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.8 0.8 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.467</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.467</iyy>
            <iyz>0</iyz>
            <izz>0.667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics properties -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

## Additional Resources

- [Gazebo Tutorials](https://gazebosim.org/tutorials)
- [SDF (Simulation Description Format)](https://gazebosim.org/api/sdf/13.3/spec.html)
- [Gazebo ROS Integration](https://gazebosim.org/docs/harmonic/ros_integration/)
- [Robot Simulation Best Practices](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html)

## Quiz Questions

1. What are the key components of a Gazebo world file?
2. How do physics parameters affect robot simulation behavior?
3. What is the difference between collision and visual models in simulation?