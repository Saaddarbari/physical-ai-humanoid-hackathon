---
sidebar_position: 2
---

# Sensor Simulation: LiDAR, Depth Cameras, IMUs

This chapter covers the simulation of various robot sensors including LiDAR, depth cameras, and IMUs in Gazebo, providing realistic sensor data for development and testing.

## Learning Goals

- Configure LiDAR sensors in Gazebo simulation
- Set up depth camera sensors for 3D perception
- Simulate IMU sensors for orientation and acceleration
- Understand the characteristics and limitations of simulated sensors

## Core Concepts

- **LiDAR Simulation**: Simulating 2D or 3D laser range finders
- **Depth Camera Simulation**: Creating realistic depth and RGB-D data
- **IMU Simulation**: Modeling inertial measurement units for pose estimation
- **Sensor Noise**: Adding realistic noise models to simulated sensors

## Implementation Section

### Adding LiDAR Sensor to Robot Model

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_with_lidar">

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.416" ixy="0" ixz="0" iyy="0.708" iyz="0" izz="1.042"/>
    </inertial>
  </link>

  <!-- LiDAR Mount -->
  <link name="lidar_mount">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.00025" ixy="0" ixz="0" iyy="0.00025" iyz="0" izz="0.000125"/>
    </inertial>
  </link>

  <!-- Joint to connect LiDAR mount to base -->
  <joint name="lidar_mount_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_mount"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR Sensor -->
  <link name="laser_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="8.33e-05" ixy="0" ixz="0" iyy="8.33e-05" iyz="0" izz="8.33e-05"/>
    </inertial>

    <!-- Gazebo plugin for LiDAR -->
    <sensor name="laser" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
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

  <!-- Joint to connect LiDAR to mount -->
  <joint name="laser_joint" type="fixed">
    <parent link="lidar_mount"/>
    <child link="laser_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

</robot>
```

### Adding Depth Camera Sensor

```xml
  <!-- Depth Camera Sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="1.1667e-06" ixy="0" ixz="0" iyy="6.6667e-07" iyz="0" izz="1.1667e-06"/>
    </inertial>

    <!-- Gazebo plugin for depth camera -->
    <sensor name="camera" type="depth">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>30</update_rate>
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
        <hack_baseline>0.07</hack_baseline>
      </plugin>
    </sensor>
  </link>

  <!-- Joint to connect camera to base -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.25 0 0.1" rpy="0 0 0"/>
  </joint>
```

### Adding IMU Sensor

```xml
  <!-- IMU Sensor -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.01 0.01 0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="8.33e-09" ixy="0" ixz="0" iyy="8.33e-09" iyz="0" izz="8.33e-09"/>
    </inertial>

    <!-- Gazebo plugin for IMU -->
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

  <!-- Joint to connect IMU to base -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
```

## Additional Resources

- [Gazebo Sensors](https://gazebosim.org/api/sdformat/13.3/sensor.html)
- [ROS 2 Sensor Integration](https://gazebosim.org/docs/harmonic/ros_interfaces/sensors/)
- [LiDAR Simulation Guide](https://gazebosim.org/tutorials?tut=ros2_lidar&cat=connect_ros)
- [Camera Simulation Guide](https://gazebosim.org/tutorials?tut=ros2_camera&cat=connect_ros)

## Quiz Questions

1. What are the key parameters that affect LiDAR sensor performance in simulation?
2. How does depth camera simulation differ from regular RGB camera simulation?
3. What factors should be considered when modeling IMU sensor noise in simulation?