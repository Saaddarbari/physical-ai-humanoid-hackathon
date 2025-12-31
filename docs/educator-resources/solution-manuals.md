---
sidebar_position: 2
---

# Solution Manuals for Exercises and Projects

This chapter provides comprehensive solution manuals for all exercises and projects in the Physical AI & Humanoid Robotics textbook, including step-by-step solutions and best practices.

## Learning Goals

- Access complete solutions for all textbook exercises
- Understand best practices for implementation
- Evaluate student work against standard solutions
- Provide guidance for complex problem-solving

## Core Concepts

- **Complete Solutions**: Step-by-step implementation guides
- **Best Practices**: Recommended approaches and coding standards
- **Alternative Solutions**: Different valid approaches to problems
- **Common Mistakes**: Typical errors and how to avoid them

## Implementation Section

### Solution Manual: ROS 2 Communication Architecture

#### Exercise 1: Simple Publisher/Subscriber

**Problem Statement**: Create a publisher that sends "Hello World" messages every 500ms and a subscriber that prints received messages.

**Complete Solution**:

```python
# Publisher Solution
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    hello_publisher = HelloWorldPublisher()

    try:
        rclpy.spin(hello_publisher)
    except KeyboardInterrupt:
        hello_publisher.get_logger().info('Shutting down publisher')
    finally:
        hello_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# Subscriber Solution
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):
    def __init__(self):
        super().__init__('hello_world_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    hello_subscriber = HelloWorldSubscriber()

    try:
        rclpy.spin(hello_subscriber)
    except KeyboardInterrupt:
        hello_subscriber.get_logger().info('Shutting down subscriber')
    finally:
        hello_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Best Practices Demonstrated**:
- Proper node initialization and cleanup
- Use of class-based node structure
- Proper logging with context
- Exception handling for graceful shutdown
- Resource cleanup in finally blocks

**Common Mistakes to Avoid**:
- Forgetting to initialize rclpy
- Not handling KeyboardInterrupt
- Missing resource cleanup
- Incorrect topic names or QoS settings

#### Exercise 2: Service Server/Client

**Problem Statement**: Create a service that adds two integers and a client that calls the service.

**Complete Solution**:

```python
# Service Server Solution
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    add_two_ints_server = AddTwoIntsServer()

    try:
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        add_two_ints_server.get_logger().info('Shutting down server')
    finally:
        add_two_ints_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# Service Client Solution
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future

def main(args=None):
    rclpy.init(args=args)

    client = AddTwoIntsClient()
    future = client.send_request(int(sys.argv[1]), int(sys.argv[2]))

    try:
        rclpy.spin_until_future_complete(client, future)
        response = future.result()
        client.get_logger().info(f'Result: {response.sum}')
    except KeyboardInterrupt:
        client.get_logger().info('Shutting down client')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Solution Manual: Robot Modeling with URDF

#### Exercise 3: Simple Mobile Robot URDF

**Problem Statement**: Create a URDF model for a simple differential drive robot with 2 wheels.

**Complete Solution**:

```xml
<?xml version="1.0"?>
<robot name="simple_diff_drive_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <origin rpy="1.57079632679 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <origin rpy="1.57079632679 0 0"/>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00005"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <origin rpy="1.57079632679 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <origin rpy="1.57079632679 0 0"/>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.00005"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

### Solution Manual: Gazebo Simulation

#### Exercise 4: Custom Gazebo World with Obstacles

**Problem Statement**: Create a Gazebo world file with a ground plane, sun, and several obstacles.

**Complete Solution**:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="custom_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a house model -->
    <model name="house">
      <pose>5 0 0 0 0 0</pose>
      <link name="house_base">
        <pose>0 0 1.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>4 4 3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>4 4 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.4 1</ambient>
            <diffuse>0.8 0.6 0.4 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>500</mass>
          <inertia>
            <ixx>1000</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1000</iyy>
            <iyz>0</iyz>
            <izz>1000</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add a cylinder obstacle -->
    <model name="cylinder_obstacle">
      <pose>-3 2 0.5 0 0 0</pose>
      <link name="cylinder_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
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
            <izz>0.625</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add a box obstacle -->
    <model name="box_obstacle">
      <pose>-3 -2 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.166</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166</iyy>
            <iyz>0</iyz>
            <izz>0.166</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

### Solution Manual: Perception Systems

#### Exercise 5: Object Detection with OpenCV

**Problem Statement**: Create a node that detects colored objects in a camera feed using OpenCV.

**Complete Solution**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')

        # Create subscription to camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        self.get_logger().info('Color detection node started')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR to HSV
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

            # Apply morphological operations to clean up the mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Process contours
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum area threshold
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)

                    # Draw bounding box
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Calculate center
                    center_x = x + w // 2
                    center_y = y + h // 2

                    # Draw center
                    cv2.circle(cv_image, (center_x, center_y), 5, (255, 0, 0), -1)

                    # Add label
                    cv2.putText(cv_image, f'Red Object ({center_x}, {center_y})',
                               (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display the result
            cv2.imshow('Color Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down color detection node')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [ROS 2 Tutorials Solutions](https://docs.ros.org/en/humble/Tutorials.html)
- [Gazebo Simulation Examples](https://gazebosim.org/tutorials)
- [Computer Vision in ROS](https://wiki.ros.org/vision_opencv)
- [Robotics Education Resources](https://www.robocup.org/)

## Quiz Questions

1. What are the key components of a complete solution for a robotics exercise?
2. How should solution manuals address different student skill levels?
3. What best practices should be emphasized in robotics programming solutions?