---
sidebar_position: 3
---

# Robot Description and Modeling

This chapter covers robot description formats, particularly URDF (Unified Robot Description Format), and how to model humanoid robots for simulation and control.

## Learning Goals

- Understand the structure and components of URDF files
- Create robot models with proper kinematic chains
- Define visual, collision, and inertial properties
- Use Xacro for more complex robot descriptions

## Core Concepts

- **URDF (Unified Robot Description Format)**: XML-based format for representing robot models
- **Kinematic Chains**: Hierarchical structure defining joint relationships
- **Visual and Collision Models**: Separate representations for visualization and physics simulation
- **Xacro**: XML macro language for creating reusable and parameterized URDFs

## Implementation Section

### Basic URDF Robot Model

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint connecting head to base -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

### Using Xacro for Parameterized Robot Model

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_width" value="0.2" />
  <xacro:property name="base_length" value="0.3" />
  <xacro:property name="base_height" value="0.1" />

  <!-- Macro for creating a wheel -->
  <xacro:macro name="simple_wheel" params="prefix parent xyz rpy radius">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <axis xyz="0 1 0"/>
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${radius}" length="0.05"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${radius}" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Use the wheel macro -->
  <xacro:simple_wheel prefix="front_left" parent="base_link" xyz="0.1 0.1 0" rpy="0 0 0" radius="0.05"/>
  <xacro:simple_wheel prefix="front_right" parent="base_link" xyz="0.1 -0.1 0" rpy="0 0 0" radius="0.05"/>
  <xacro:simple_wheel prefix="back_left" parent="base_link" xyz="-0.1 0.1 0" rpy="0 0 0" radius="0.05"/>
  <xacro:simple_wheel prefix="back_right" parent="base_link" xyz="-0.1 -0.1 0" rpy="0 0 0" radius="0.05"/>

</robot>
```

## Additional Resources

- [URDF Documentation](https://wiki.ros.org/urdf)
- [Xacro Documentation](https://wiki.ros.org/xacro)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/)
- [Robot Modeling Best Practices](https://ros.org/reps/rep-0103.html)

## Quiz Questions

1. What is the difference between visual and collision models in URDF?
2. Why is it important to define proper inertial properties in robot models?
3. How does Xacro help in creating complex robot descriptions?