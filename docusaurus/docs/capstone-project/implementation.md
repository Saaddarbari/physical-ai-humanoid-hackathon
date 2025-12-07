---
sidebar_position: 4
---

# Step-by-Step Capstone Implementation

This chapter provides a detailed walkthrough of implementing the complete voice-controlled object manipulation system, covering all integration aspects and practical considerations.

## Learning Goals

- Implement the complete voice-to-manipulation pipeline
- Integrate all subsystems with proper error handling
- Validate system performance and reliability
- Deploy the complete system on a humanoid robot platform

## Core Concepts

- **System Architecture**: Understanding the complete system design
- **Component Integration**: Connecting all modules effectively
- **Performance Validation**: Testing system performance metrics
- **Deployment Considerations**: Practical aspects of real-world deployment

## Implementation Section

### Complete System Launch File

```xml
<!-- capstone_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_model = LaunchConfiguration('robot_model', default='humanoid_robot')

    # Voice command processing nodes
    voice_recognition_node = Node(
        package='capstone_demo',
        executable='voice_recognition_node',
        name='voice_recognition',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    nlu_processor_node = Node(
        package='capstone_demo',
        executable='nlu_processor_node',
        name='nlu_processor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Perception nodes
    object_detection_node = Node(
        package='capstone_demo',
        executable='object_detection_node',
        name='object_detection',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation nodes
    navigation_node = Node(
        package='capstone_demo',
        executable='navigation_node',
        name='navigation',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Manipulation nodes
    grasp_planner_node = Node(
        package='capstone_demo',
        executable='grasp_planner_node',
        name='grasp_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Integration nodes
    integration_node = Node(
        package='capstone_demo',
        executable='integration_node',
        name='integration',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    task_planner_node = Node(
        package='capstone_demo',
        executable='task_planner_node',
        name='task_planner',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('capstone_demo'), 'config', 'capstone_system.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz', default='true'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('robot_model', default_value='humanoid_robot', description='Robot model to use'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),

        # Launch all nodes
        voice_recognition_node,
        nlu_processor_node,
        object_detection_node,
        navigation_node,
        grasp_planner_node,
        integration_node,
        task_planner_node,
        rviz_node
    ])
```

### System Configuration File

```yaml
# config/capstone_system.yaml
/**:
  ros__parameters:
    use_sim_time: false

voice_recognition:
  ros__parameters:
    sample_rate: 16000
    chunk_size: 1024
    energy_threshold: 4000
    dynamic_energy_threshold: true

object_detection:
  ros__parameters:
    detection_threshold: 0.5
    max_objects: 10
    tracking_enabled: true

navigation:
  ros__parameters:
    planner_frequency: 5.0
    controller_frequency: 20.0
    recovery_enabled: true
    max_vel_x: 0.5
    min_vel_x: 0.1
    max_vel_theta: 1.0

grasp_planner:
  ros__parameters:
    max_grasps: 10
    grasp_distance: 0.15
    approach_distance: 0.05
    lift_height: 0.1

integration:
  ros__parameters:
    state_machine_rate: 10.0
    timeout_navigation: 30.0
    timeout_perception: 10.0
    timeout_grasping: 15.0
    max_retries: 3

task_planner:
  ros__parameters:
    queue_size: 100
    priority_based: true
    preemption_enabled: true
```

### System Performance Monitoring Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time
import time
import psutil
import threading
from collections import deque
import statistics

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')

        # Create publishers for system metrics
        self.cpu_publisher = self.create_publisher(Float32, '/diagnostics/cpu_usage', 10)
        self.memory_publisher = self.create_publisher(Float32, '/diagnostics/memory_usage', 10)
        self.task_success_publisher = self.create_publisher(Float32, '/diagnostics/task_success_rate', 10)
        self.response_time_publisher = self.create_publisher(Float32, '/diagnostics/response_time', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Create subscriptions for task monitoring
        self.task_status_subscription = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            10
        )

        # Initialize monitoring data
        self.task_history = deque(maxlen=100)  # Keep last 100 tasks
        self.response_times = deque(maxlen=50)  # Keep last 50 response times
        self.start_times = {}  # Track start times for active tasks

        # Start monitoring timer
        self.monitor_timer = self.create_timer(1.0, self.publish_diagnostics)

        self.get_logger().info('System monitor node started')

    def task_status_callback(self, msg):
        """Monitor task status and calculate success metrics"""
        status = msg.data
        timestamp = self.get_clock().now().nanoseconds / 1e9

        if status.startswith('start_'):
            # Task started
            task_name = status[6:]  # Remove 'start_' prefix
            self.start_times[task_name] = timestamp
        elif status.startswith('complete_'):
            # Task completed successfully
            task_name = status[9:]  # Remove 'complete_' prefix
            if task_name in self.start_times:
                response_time = timestamp - self.start_times[task_name]
                self.response_times.append(response_time)
                self.task_history.append(('success', task_name, response_time))
                del self.start_times[task_name]
        elif status.startswith('fail_'):
            # Task failed
            task_name = status[5:]  # Remove 'fail_' prefix
            if task_name in self.start_times:
                response_time = timestamp - self.start_times[task_name]
                self.task_history.append(('failure', task_name, response_time))
                del self.start_times[task_name]

    def publish_diagnostics(self):
        """Publish system diagnostic information"""
        # Create diagnostic array
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # System resource diagnostics
        cpu_diag = DiagnosticStatus()
        cpu_diag.name = 'System Monitor/CPU Usage'
        cpu_percent = psutil.cpu_percent()
        cpu_diag.values = [KeyValue(key='CPU Percent', value=str(cpu_percent))]

        if cpu_percent < 70:
            cpu_diag.level = DiagnosticStatus.OK
            cpu_diag.message = f'CPU usage normal: {cpu_percent}%'
        elif cpu_percent < 90:
            cpu_diag.level = DiagnosticStatus.WARN
            cpu_diag.message = f'CPU usage high: {cpu_percent}%'
        else:
            cpu_diag.level = DiagnosticStatus.ERROR
            cpu_diag.message = f'CPU usage critical: {cpu_percent}%'

        diag_array.status.append(cpu_diag)

        # Memory diagnostics
        memory_diag = DiagnosticStatus()
        memory_diag.name = 'System Monitor/Memory Usage'
        memory_percent = psutil.virtual_memory().percent
        memory_diag.values = [KeyValue(key='Memory Percent', value=str(memory_percent))]

        if memory_percent < 70:
            memory_diag.level = DiagnosticStatus.OK
            memory_diag.message = f'Memory usage normal: {memory_percent}%'
        elif memory_percent < 90:
            memory_diag.level = DiagnosticStatus.WARN
            memory_diag.message = f'Memory usage high: {memory_percent}%'
        else:
            memory_diag.level = DiagnosticStatus.ERROR
            memory_diag.message = f'Memory usage critical: {memory_percent}%'

        diag_array.status.append(memory_diag)

        # Task performance diagnostics
        if self.task_history:
            successful_tasks = [t for t in self.task_history if t[0] == 'success']
            total_tasks = len(self.task_history)
            success_rate = len(successful_tasks) / total_tasks if total_tasks > 0 else 0

            task_diag = DiagnosticStatus()
            task_diag.name = 'System Monitor/Task Success Rate'
            task_diag.values = [
                KeyValue(key='Successful Tasks', value=str(len(successful_tasks))),
                KeyValue(key='Total Tasks', value=str(total_tasks)),
                KeyValue(key='Success Rate', value=f'{success_rate:.2%}')
            ]

            if success_rate >= 0.9:
                task_diag.level = DiagnosticStatus.OK
                task_diag.message = f'High task success rate: {success_rate:.2%}'
            elif success_rate >= 0.7:
                task_diag.level = DiagnosticStatus.WARN
                task_diag.message = f'Moderate task success rate: {success_rate:.2%}'
            else:
                task_diag.level = DiagnosticStatus.ERROR
                task_diag.message = f'Low task success rate: {success_rate:.2%}'

            diag_array.status.append(task_diag)

        # Response time diagnostics
        if self.response_times:
            avg_response_time = statistics.mean(self.response_times)
            response_diag = DiagnosticStatus()
            response_diag.name = 'System Monitor/Average Response Time'
            response_diag.values = [
                KeyValue(key='Average Response Time', value=f'{avg_response_time:.3f}s'),
                KeyValue(key='Sample Count', value=str(len(self.response_times)))
            ]

            if avg_response_time < 2.0:
                response_diag.level = DiagnosticStatus.OK
                response_diag.message = f'Fast response time: {avg_response_time:.3f}s'
            elif avg_response_time < 5.0:
                response_diag.level = DiagnosticStatus.WARN
                response_diag.message = f'Moderate response time: {avg_response_time:.3f}s'
            else:
                response_diag.level = DiagnosticStatus.ERROR
                response_diag.message = f'Slow response time: {avg_response_time:.3f}s'

            diag_array.status.append(response_diag)

        # Publish diagnostics
        self.diagnostics_publisher.publish(diag_array)

        # Publish individual metrics for monitoring
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_publisher.publish(cpu_msg)

        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_publisher.publish(memory_msg)

        success_msg = Float32()
        success_msg.data = float(success_rate)
        self.task_success_publisher.publish(success_msg)

        if self.response_times:
            response_msg = Float32()
            response_msg.data = float(avg_response_time)
            self.response_time_publisher.publish(response_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down system monitor node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Deployment and Testing Script

```python
#!/usr/bin/env python3
# deployment_test.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import sys
import subprocess

class DeploymentTestNode(Node):
    def __init__(self):
        super().__init__('deployment_test_node')

        # Create publishers for testing
        self.voice_cmd_publisher = self.create_publisher(String, '/voice_command', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Start test sequence
        self.timer = self.create_timer(5.0, self.run_test_sequence)
        self.test_step = 0

        self.get_logger().info('Deployment test node started')

    def run_test_sequence(self):
        """Run a sequence of tests to validate the system"""
        tests = [
            self.test_voice_recognition,
            self.test_navigation,
            self.test_object_detection,
            self.test_grasping_simulation,
            self.test_integration
        ]

        if self.test_step < len(tests):
            self.get_logger().info(f'Running test {self.test_step + 1}: {tests[self.test_step].__name__}')
            tests[self.test_step]()
            self.test_step += 1
        else:
            self.get_logger().info('All deployment tests completed')
            self.timer.cancel()

    def test_voice_recognition(self):
        """Test voice recognition system"""
        self.get_logger().info('Testing voice recognition...')

        # Simulate voice commands
        commands = [
            "move forward",
            "turn left",
            "stop",
            "pick up red block"
        ]

        for cmd in commands:
            voice_msg = String()
            voice_msg.data = cmd
            self.voice_cmd_publisher.publish(voice_msg)
            self.get_logger().info(f'Sent voice command: {cmd}')
            time.sleep(1)

    def test_navigation(self):
        """Test navigation system"""
        self.get_logger().info('Testing navigation system...')

        # Send simple navigation commands
        cmd = Twist()
        cmd.linear.x = 0.3  # Move forward
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(2)  # Move for 2 seconds

        # Stop
        cmd.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(1)

        # Turn
        cmd.angular.z = 0.5  # Turn left
        self.cmd_vel_publisher.publish(cmd)
        time.sleep(2)  # Turn for 2 seconds

        # Stop
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

    def test_object_detection(self):
        """Test object detection system"""
        self.get_logger().info('Testing object detection...')

        # In a real test, this would validate that objects are properly detected
        # For simulation, we just wait and verify the system is running
        time.sleep(5)

    def test_grasping_simulation(self):
        """Test grasping system (simulation)"""
        self.get_logger().info('Testing grasping system...')

        # In a real system, this would test the actual grasping mechanism
        # For simulation, we just wait
        time.sleep(5)

    def test_integration(self):
        """Test full system integration"""
        self.get_logger().info('Testing full system integration...')

        # Send a complex command that requires full system integration
        voice_msg = String()
        voice_msg.data = "pick up the red block and bring it to me"
        self.voice_cmd_publisher.publish(voice_msg)

        self.get_logger().info('Full integration test started - monitor system response')
        time.sleep(10)  # Allow time for full sequence

def main(args=None):
    rclpy.init(args=args)
    node = DeploymentTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down deployment test node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [ROS 2 Deployment Guide](https://docs.ros.org/en/humble/How-To-Guides/Deployment.html)
- [System Integration Testing](https://arxiv.org/abs/1903.09842)
- [Robot Performance Monitoring](https://ieeexplore.ieee.org/document/8206289)
- [Real-world Robotics Deployment](https://www.sciencedirect.com/science/article/pii/S0921889017304222)

## Quiz Questions

1. What are the key components of a complete voice-controlled manipulation system?
2. How should system performance be monitored in real-world deployment?
3. What testing strategies are important for validating integrated robotics systems?