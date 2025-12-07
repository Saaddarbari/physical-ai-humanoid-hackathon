---
sidebar_position: 1
---

# Physics Simulation and Environment Building

This chapter covers the fundamentals of physics simulation using Gazebo, including environment creation, physics properties, and simulation setup for humanoid robotics.

## Learning Goals

- Create realistic simulation environments for humanoid robots
- Configure physics properties and parameters
- Set up sensors and actuators in simulation
- Understand the differences between simulation and real-world physics

## Core Concepts

- **Gazebo Simulation**: Physics-based simulation environment for robotics
- **World Building**: Creating environments with proper physics properties
- **Simulation Parameters**: Configuring gravity, friction, and other physics properties
- **Sensor Integration**: Adding sensors to simulation for realistic data

## Implementation Section

### Creating a Gazebo World File

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
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
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add a table -->
    <model name="table">
      <pose>-2 1 0.5 0 0 0</pose>
      <link name="table_top">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 1 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.5 1 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.25</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>2.8125</iyy>
            <iyz>0</iyz>
            <izz>4.0625</izz>
          </inertia>
        </inertial>
      </link>
      <!-- Table legs -->
      <link name="leg1">
        <pose>-0.6 -0.4 0 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.3 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.084167</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.084167</iyy>
            <iyz>0</iyz>
            <izz>0.008333</izz>
          </inertia>
        </inertial>
      </link>
      <joint name="leg1_joint" type="fixed">
        <parent>table_top</parent>
        <child>leg1</child>
      </joint>
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

### Launching Gazebo with a Robot Model

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Create publisher for robot velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Create timer for periodic control commands
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Simulation controller started')

    def control_loop(self):
        msg = Twist()

        # Simple oscillating motion
        time = self.get_clock().now().nanoseconds / 1e9
        msg.linear.x = 0.5 * math.sin(time)  # Forward/back motion
        msg.angular.z = 0.3 * math.cos(time)  # Turning motion

        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SimulationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down simulation controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [Gazebo Tutorials](https://gazebosim.org/tutorials)
- [SDF (Simulation Description Format)](https://gazebosim.org/docs/harmonic/sdf/)
- [Gazebo Physics](https://gazebosim.org/api/gazebo/6/physics.html)
- [ROS 2 Gazebo Integration](https://gazebosim.org/docs/harmonic/ros_integration/)

## Quiz Questions

1. What are the key components of a Gazebo world file?
2. How do physics parameters affect robot simulation behavior?
3. What is the difference between collision and visual models in simulation?