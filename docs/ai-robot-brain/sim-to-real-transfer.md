---
sidebar_position: 3
---

# Sim-to-Real Transfer Techniques

This chapter covers techniques for transferring robot behaviors and AI models from simulation to real-world deployment, addressing the reality gap and domain randomization.

## Learning Goals

- Understand the challenges of sim-to-real transfer
- Apply domain randomization techniques to improve transferability
- Implement system identification for model correction
- Evaluate and validate sim-to-real performance

## Core Concepts

- **Reality Gap**: Differences between simulation and real-world behavior
- **Domain Randomization**: Randomizing simulation parameters to improve robustness
- **System Identification**: Modeling real robot dynamics for better simulation
- **Adaptive Control**: Adjusting behavior based on real-world feedback

## Implementation Section

### Domain Randomization Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import random
import math

class DomainRandomizationNode(Node):
    def __init__(self):
        super().__init__('domain_randomization_node')

        # Create subscription to joint states
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create publisher for motor commands
        self.motor_command_publisher = self.create_publisher(
            JointState,
            '/motor_commands',
            10
        )

        # Create timer for randomization
        self.randomization_timer = self.create_timer(5.0, self.randomize_simulation_params)

        # Initialize simulation parameters with randomization ranges
        self.base_params = {
            'mass': 10.0,  # Base mass of robot
            'friction': 0.1,  # Base friction coefficient
            'inertia': [1.0, 1.0, 1.0]  # Base inertia values
        }

        # Randomization ranges
        self.randomization_ranges = {
            'mass_range': (0.8, 1.2),      # ±20% mass variation
            'friction_range': (0.05, 0.3), # Friction from 0.05 to 0.3
            'inertia_range': (0.7, 1.3)    # ±30% inertia variation
        }

        # Current randomized parameters
        self.current_params = self.randomize_parameters()

        self.get_logger().info('Domain randomization node started')

    def randomize_parameters(self):
        """Randomize simulation parameters within defined ranges"""
        randomized_params = {}

        # Randomize mass
        mass_factor = random.uniform(
            self.randomization_ranges['mass_range'][0],
            self.randomization_ranges['mass_range'][1]
        )
        randomized_params['mass'] = self.base_params['mass'] * mass_factor

        # Randomize friction
        randomized_params['friction'] = random.uniform(
            self.randomization_ranges['friction_range'][0],
            self.randomization_ranges['friction_range'][1]
        )

        # Randomize inertia
        inertia_factors = [
            random.uniform(
                self.randomization_ranges['inertia_range'][0],
                self.randomization_ranges['inertia_range'][1]
            ) for _ in range(3)
        ]
        randomized_params['inertia'] = [
            self.base_params['inertia'][i] * inertia_factors[i]
            for i in range(3)
        ]

        return randomized_params

    def randomize_simulation_params(self):
        """Randomize simulation parameters periodically"""
        self.current_params = self.randomize_parameters()

        self.get_logger().info(
            f'Randomized parameters: mass={self.current_params["mass"]:.2f}, '
            f'friction={self.current_params["friction"]:.2f}, '
            f'inertia={self.current_params["inertia"]}'
        )

    def joint_state_callback(self, msg):
        # In simulation, we would apply the randomized parameters here
        # For example, we might adjust control gains based on the current parameters

        # Calculate adjusted control parameters based on randomized values
        mass_factor = self.current_params['mass'] / self.base_params['mass']
        friction_factor = self.current_params['friction'] / self.base_params['friction']

        # Create and publish motor commands with adaptive control
        motor_cmd = JointState()
        motor_cmd.header.stamp = self.get_clock().now().to_msg()
        motor_cmd.name = msg.name
        motor_cmd.position = msg.position  # Copy current positions
        motor_cmd.velocity = [0.0] * len(msg.name)  # Set desired velocities
        motor_cmd.effort = [0.0] * len(msg.name)    # Set desired efforts

        # Adjust control based on randomized parameters
        # (In a real implementation, this would involve more complex control logic)
        for i in range(len(motor_cmd.effort)):
            # Simulate adaptive control based on randomized parameters
            base_effort = 1.0  # Base effort value
            adjusted_effort = base_effort * mass_factor * friction_factor
            motor_cmd.effort[i] = adjusted_effort

        self.motor_command_publisher.publish(motor_cmd)

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

### System Identification Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.optimize import minimize
import math

class SystemIdentificationNode(Node):
    def __init__(self):
        super().__init__('system_identification_node')

        # Create subscriptions for sensor data
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publisher for identified parameters
        self.param_publisher = self.create_publisher(
            Float64MultiArray,
            '/identified_parameters',
            10
        )

        # Data collection variables
        self.joint_data_buffer = []
        self.imu_data_buffer = []
        self.max_buffer_size = 1000

        # Robot model parameters (initial estimates)
        self.identified_params = {
            'mass': 10.0,           # Robot mass
            'com_x': 0.0,           # Center of mass x offset
            'com_y': 0.0,           # Center of mass y offset
            'com_z': 0.5,           # Center of mass z offset
            'inertia_xx': 1.0,      # Moments of inertia
            'inertia_yy': 1.0,
            'inertia_zz': 1.0,
            'motor_constant': 0.1,  # Motor constant
            'friction_coeff': 0.01  # Friction coefficient
        }

        # Create timer for parameter estimation
        self.estimation_timer = self.create_timer(10.0, self.estimate_parameters)

        self.get_logger().info('System identification node started')

    def joint_state_callback(self, msg):
        """Collect joint state data for system identification"""
        # Store relevant data: position, velocity, effort
        data_point = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'position': list(msg.position),
            'velocity': list(msg.velocity),
            'effort': list(msg.effort)
        }

        self.joint_data_buffer.append(data_point)

        # Maintain buffer size
        if len(self.joint_data_buffer) > self.max_buffer_size:
            self.joint_data_buffer.pop(0)

    def imu_callback(self, msg):
        """Collect IMU data for system identification"""
        # Store IMU data: orientation, angular velocity, linear acceleration
        data_point = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'orientation': [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ],
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
        }

        self.imu_data_buffer.append(data_point)

        # Maintain buffer size
        if len(self.imu_data_buffer) > self.max_buffer_size:
            self.imu_data_buffer.pop(0)

    def estimate_parameters(self):
        """Estimate robot parameters using collected data"""
        if len(self.joint_data_buffer) < 100:  # Need sufficient data
            self.get_logger().info('Not enough data for parameter estimation')
            return

        try:
            # Prepare data for optimization
            # This is a simplified example - real system ID would be more complex
            t_data = []
            pos_data = []
            vel_data = []
            acc_data = []
            effort_data = []

            # Extract and align data (simplified)
            for i in range(min(len(self.joint_data_buffer), len(self.imu_data_buffer)) - 1):
                joint_now = self.joint_data_buffer[i+1]
                joint_prev = self.joint_data_buffer[i]

                # Calculate acceleration (simplified)
                dt = joint_now['timestamp'] - joint_prev['timestamp']
                if dt > 0:
                    acc = [(v_now - v_prev) / dt for v_now, v_prev in
                           zip(joint_now['velocity'], joint_prev['velocity'])]

                    t_data.append(joint_now['timestamp'])
                    pos_data.append(joint_now['position'])
                    vel_data.append(joint_now['velocity'])
                    acc_data.append(acc)
                    effort_data.append(joint_now['effort'])

            if len(acc_data) < 10:
                self.get_logger().info('Insufficient valid acceleration data')
                return

            # Perform parameter estimation (simplified approach)
            # In reality, this would involve more complex dynamics modeling
            estimated_params = self.simple_parameter_estimation(
                pos_data, vel_data, acc_data, effort_data
            )

            # Update identified parameters
            self.identified_params.update(estimated_params)

            # Publish identified parameters
            param_msg = Float64MultiArray()
            param_msg.data = [
                self.identified_params['mass'],
                self.identified_params['com_x'],
                self.identified_params['com_y'],
                self.identified_params['com_z'],
                self.identified_params['inertia_xx'],
                self.identified_params['inertia_yy'],
                self.identified_params['inertia_zz'],
                self.identified_params['motor_constant'],
                self.identified_params['friction_coeff']
            ]

            self.param_publisher.publish(param_msg)

            self.get_logger().info(f'Updated parameters: mass={self.identified_params["mass"]:.3f}, '
                                 f'com_z={self.identified_params["com_z"]:.3f}')

        except Exception as e:
            self.get_logger().error(f'Error in parameter estimation: {e}')

    def simple_parameter_estimation(self, pos_data, vel_data, acc_data, effort_data):
        """Simplified parameter estimation - in reality this would be more complex"""
        # This is a placeholder for a real system identification algorithm
        # which would solve the inverse dynamics problem to identify parameters

        # For demonstration, we'll just make some adjustments based on observed behavior
        estimated = {}

        # Estimate mass based on effort vs acceleration relationship
        if acc_data and effort_data:
            # Simplified mass estimation (tau = I*alpha + friction + gravity*mass*com)
            avg_effort = np.mean([np.mean(efforts) for efforts in effort_data[-50:]])
            avg_acc = np.mean([np.mean(np.abs(accs)) for accs in acc_data[-50:]])

            if avg_acc > 0.001:  # Avoid division by zero
                # Rough estimation: mass ≈ effort / (inertia * acceleration)
                estimated['mass'] = max(1.0, min(50.0, avg_effort / avg_acc))
            else:
                estimated['mass'] = self.identified_params['mass']

        # Estimate center of mass based on IMU data patterns
        # (This would require more sophisticated analysis in practice)
        estimated['com_z'] = self.identified_params['com_z']  # Keep as is for now

        # Other parameters remain unchanged in this simplified example
        for param in ['com_x', 'com_y', 'inertia_xx', 'inertia_yy', 'inertia_zz',
                      'motor_constant', 'friction_coeff']:
            estimated[param] = self.identified_params[param]

        return estimated

def main(args=None):
    rclpy.init(args=args)
    node = SystemIdentificationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down system identification node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [Domain Randomization Papers](https://arxiv.org/abs/1703.06907)
- [Sim-to-Real Transfer Techniques](https://openreview.net/forum?id=Hk0dUMb0Z)
- [System Identification for Robotics](https://ieeexplore.ieee.org/document/9123456)
- [NVIDIA Isaac Sim Documentation](https://docs.nvidia.com/isaac/)

## Quiz Questions

1. What is the "reality gap" in sim-to-real transfer?
2. How does domain randomization help improve transferability?
3. What are the key challenges in system identification for humanoid robots?