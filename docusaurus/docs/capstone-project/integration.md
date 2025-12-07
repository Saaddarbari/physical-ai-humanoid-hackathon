---
sidebar_position: 3
---

# Integration: Voice Command to Object Manipulation

This chapter covers the integration of all system components to create a complete voice-controlled object manipulation system, connecting voice processing, perception, navigation, and manipulation modules.

## Learning Goals

- Integrate voice command processing with robot navigation and manipulation
- Create a state machine for managing complex multi-step tasks
- Implement error handling and recovery for integrated systems
- Validate the complete voice-to-action pipeline

## Core Concepts

- **System Integration**: Connecting different robotics modules into a cohesive system
- **State Management**: Managing the robot's state during complex tasks
- **Error Recovery**: Handling failures and returning to safe states
- **Task Planning**: Coordinating multiple actions to achieve complex goals

## Implementation Section

### Main Integration Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes
from std_srvs.srv import Trigger
import time
import threading
from enum import Enum

class RobotState(Enum):
    IDLE = 1
    LISTENING = 2
    NAVIGATING = 3
    PERCEIVING = 4
    GRASPING = 5
    MANIPULATING = 6
    ERROR = 7

class IntegrationNode(Node):
    def __init__(self):
        super().__init__('integration_node')

        # Create subscriptions
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )

        self.robot_state_subscription = self.create_subscription(
            String,
            '/robot_state',
            self.robot_state_callback,
            10
        )

        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_status_publisher = self.create_publisher(String, '/task_status', 10)
        self.robot_state_publisher = self.create_publisher(String, '/robot_state', 10)

        # Create service clients
        self.nav_client = self.create_client(Trigger, '/navigate_to_object')
        self.perception_client = self.create_client(Trigger, '/detect_object')
        self.grasp_client = self.create_client(Trigger, '/grasp_object')

        # Initialize state
        self.current_state = RobotState.IDLE
        self.target_object = None
        self.navigation_goal_reached = False
        self.object_detected = False
        self.object_grasped = False

        # Start state machine
        self.state_machine_thread = threading.Thread(target=self.state_machine)
        self.state_machine_thread.daemon = True
        self.state_machine_thread.start()

        self.get_logger().info('Integration node started')

    def voice_command_callback(self, msg):
        """Process voice commands and update robot state"""
        command = msg.data.lower()

        self.get_logger().info(f'Received voice command: {command}')

        # Update state based on command
        if 'pick up' in command or 'grasp' in command or 'get' in command:
            if 'red' in command or 'blue' in command or 'green' in command:
                self.target_object = command.split()[-1]  # Get color
                self.current_state = RobotState.NAVIGATING
                self.publish_robot_state()
        elif 'move to' in command or 'go to' in command:
            self.current_state = RobotState.NAVIGATING
            self.publish_robot_state()
        elif 'stop' in command or 'halt' in command:
            self.current_state = RobotState.IDLE
            self.publish_robot_state()

    def robot_state_callback(self, msg):
        """Update internal state from other nodes"""
        state_str = msg.data
        try:
            self.current_state = RobotState[state_str.upper()]
        except KeyError:
            self.get_logger().warn(f'Unknown robot state: {state_str}')

    def publish_robot_state(self):
        """Publish current robot state"""
        state_msg = String()
        state_msg.data = self.current_state.name.lower()
        self.robot_state_publisher.publish(state_msg)

    def state_machine(self):
        """Main state machine for integrated behavior"""
        while rclpy.ok():
            try:
                if self.current_state == RobotState.IDLE:
                    # In idle state, just listen for commands
                    time.sleep(0.1)

                elif self.current_state == RobotState.LISTENING:
                    # Wait for voice command
                    time.sleep(0.1)

                elif self.current_state == RobotState.NAVIGATING:
                    self.execute_navigation()
                    if self.navigation_goal_reached:
                        self.current_state = RobotState.PERCEIVING
                        self.publish_robot_state()

                elif self.current_state == RobotState.PERCEIVING:
                    self.execute_perception()
                    if self.object_detected:
                        self.current_state = RobotState.GRASPING
                        self.publish_robot_state()

                elif self.current_state == RobotState.GRASPING:
                    self.execute_grasping()
                    if self.object_grasped:
                        self.current_state = RobotState.MANIPULATING
                        self.publish_robot_state()
                    else:
                        # Grasping failed, go back to perception or report error
                        self.current_state = RobotState.ERROR
                        self.publish_robot_state()

                elif self.current_state == RobotState.MANIPULATING:
                    # In a complete system, this would involve moving the object
                    # For this example, we'll just report success and return to idle
                    self.get_logger().info('Object manipulation completed successfully')
                    self.current_state = RobotState.IDLE
                    self.publish_robot_state()

                elif self.current_state == RobotState.ERROR:
                    self.handle_error()
                    self.current_state = RobotState.IDLE
                    self.publish_robot_state()

                time.sleep(0.1)  # Small delay to prevent busy waiting

            except Exception as e:
                self.get_logger().error(f'Error in state machine: {e}')
                self.current_state = RobotState.ERROR
                self.publish_robot_state()
                time.sleep(1.0)

    def execute_navigation(self):
        """Execute navigation to target location"""
        self.get_logger().info('Executing navigation...')

        # In a real system, this would call navigation services
        # For this example, we'll simulate navigation completion
        time.sleep(2.0)  # Simulate navigation time
        self.navigation_goal_reached = True
        self.get_logger().info('Navigation completed')

    def execute_perception(self):
        """Execute object perception"""
        self.get_logger().info('Executing perception...')

        # In a real system, this would call perception services
        # For this example, we'll simulate object detection
        time.sleep(1.5)  # Simulate perception time
        self.object_detected = True
        self.get_logger().info('Object detected')

    def execute_grasping(self):
        """Execute grasping action"""
        self.get_logger().info('Executing grasping...')

        # In a real system, this would call grasping services
        # For this example, we'll simulate grasping with 80% success rate
        import random
        success = random.random() < 0.8  # 80% success rate
        time.sleep(2.0)  # Simulate grasping time

        if success:
            self.object_grasped = True
            self.get_logger().info('Object grasped successfully')
        else:
            self.object_grasped = False
            self.get_logger().warn('Grasping failed')

    def handle_error(self):
        """Handle error state"""
        self.get_logger().error('Error state reached, stopping robot')

        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

        # Reset flags
        self.navigation_goal_reached = False
        self.object_detected = False
        self.object_grasped = False

        self.get_logger().info('Error recovery completed')

def main(args=None):
    rclpy.init(args=args)
    node = IntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down integration node')
    finally:
        # Stop robot on shutdown
        stop_cmd = Twist()
        node.cmd_vel_publisher.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Task Planning and Coordination Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Pose
from action_msgs.msg import GoalStatus
from std_srvs.srv import Trigger
import time
from enum import Enum
from collections import deque

class TaskType(Enum):
    NAVIGATE = 1
    PERCEIVE = 2
    GRASP = 3
    MANIPULATE = 4
    SPEAK = 5

class Task:
    def __init__(self, task_type, params=None, priority=1):
        self.type = task_type
        self.params = params or {}
        self.priority = priority
        self.created_time = time.time()
        self.status = "pending"  # pending, executing, completed, failed

    def __repr__(self):
        return f"Task({self.type.name}, priority={self.priority}, status={self.status})"

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        # Create subscriptions
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )

        self.task_status_subscription = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            10
        )

        # Create publishers
        self.task_queue_publisher = self.create_publisher(String, '/next_task', 10)
        self.speech_publisher = self.create_publisher(String, '/speech_output', 10)

        # Create service clients for different capabilities
        self.nav_client = self.create_client(Trigger, '/navigate_service')
        self.perception_client = self.create_client(Trigger, '/perception_service')
        self.grasp_client = self.create_client(Trigger, '/grasp_service')

        # Task queue (priority queue)
        self.task_queue = deque()
        self.current_task = None

        # Object properties (in a real system, this would come from perception)
        self.known_objects = {
            'red block': {'color': 'red', 'type': 'block', 'location': Pose()},
            'blue cup': {'color': 'blue', 'type': 'cup', 'location': Pose()},
            'green ball': {'color': 'green', 'type': 'ball', 'location': Pose()},
        }

        self.get_logger().info('Task planner node started')

    def voice_command_callback(self, msg):
        """Parse voice command and create tasks"""
        command = msg.data.lower()
        self.get_logger().info(f'Planning tasks for command: {command}')

        # Parse command and create appropriate task sequence
        tasks = self.parse_command_to_tasks(command)

        # Add tasks to queue
        for task in tasks:
            self.add_task(task)

    def parse_command_to_tasks(self, command):
        """Parse a voice command into a sequence of tasks"""
        tasks = []

        if 'pick up' in command or 'get me' in command or 'grasp' in command:
            # Extract object information
            target_object = self.extract_object_from_command(command)

            # Create task sequence: navigate -> perceive -> grasp
            if target_object:
                tasks.append(Task(TaskType.NAVIGATE, {'target_object': target_object}))
                tasks.append(Task(TaskType.PERCEIVE, {'target_object': target_object}))
                tasks.append(Task(TaskType.GRASP, {'target_object': target_object}))

                # Announce task sequence
                self.announce_task_sequence(tasks)

        elif 'move to' in command or 'go to' in command:
            # Extract location information
            target_location = self.extract_location_from_command(command)

            if target_location:
                tasks.append(Task(TaskType.NAVIGATE, {'target_location': target_location}))

                # Announce task
                self.announce_task_sequence(tasks)

        return tasks

    def extract_object_from_command(self, command):
        """Extract object information from command"""
        # Simple keyword matching (in reality, use NLP)
        for obj_name in self.known_objects.keys():
            if obj_name in command:
                return obj_name

        # Try to match by color + type
        colors = ['red', 'blue', 'green', 'yellow', 'white', 'black']
        types = ['block', 'cup', 'ball', 'box', 'object']

        found_color = None
        found_type = None

        for color in colors:
            if color in command:
                found_color = color
                break

        for obj_type in types:
            if obj_type in command:
                found_type = obj_type
                break

        if found_color and found_type:
            return f"{found_color} {found_type}"
        elif found_color:
            return found_color
        elif found_type:
            return found_type

        return "unknown object"

    def extract_location_from_command(self, command):
        """Extract location information from command"""
        # Simple location extraction (in reality, use semantic mapping)
        locations = {
            'kitchen': (3.0, 1.0, 0.0),
            'living room': (0.0, 2.0, 0.0),
            'bedroom': (-2.0, -1.0, 0.0),
            'bathroom': (1.0, -2.0, 0.0),
        }

        for location_name, coordinates in locations.items():
            if location_name in command:
                pose = Pose()
                pose.position.x = coordinates[0]
                pose.position.y = coordinates[1]
                pose.position.z = coordinates[2]
                return pose

        return None

    def add_task(self, task):
        """Add a task to the queue with priority ordering"""
        # Insert task based on priority (higher priority first)
        inserted = False
        for i, existing_task in enumerate(self.task_queue):
            if task.priority > existing_task.priority:
                self.task_queue.insert(i, task)
                inserted = True
                break

        if not inserted:
            self.task_queue.append(task)

        self.get_logger().info(f'Added task: {task}')

    def announce_task_sequence(self, tasks):
        """Announce the planned task sequence"""
        task_names = [task.type.name for task in tasks]
        announcement = f"Planning to execute tasks: {', '.join(task_names)}"

        speech_msg = String()
        speech_msg.data = announcement
        self.speech_publisher.publish(speech_msg)

        self.get_logger().info(announcement)

    def task_status_callback(self, msg):
        """Handle task status updates"""
        status = msg.data
        self.get_logger().info(f'Task status update: {status}')

        # Process next task if current task is completed
        if status == 'completed' and self.current_task:
            self.current_task.status = 'completed'
            self.current_task = None
            self.process_next_task()
        elif status == 'failed' and self.current_task:
            self.current_task.status = 'failed'
            self.current_task = None
            self.handle_task_failure()

    def process_next_task(self):
        """Process the next task in the queue"""
        if self.task_queue:
            self.current_task = self.task_queue.popleft()
            self.current_task.status = 'executing'

            self.get_logger().info(f'Executing task: {self.current_task}')

            # Publish the task for execution
            task_msg = String()
            task_msg.data = self.current_task.type.name.lower()
            self.task_queue_publisher.publish(task_msg)

    def handle_task_failure(self):
        """Handle task failure and plan recovery"""
        self.get_logger().error('Task failed, planning recovery...')

        # For now, just try the same task again (in reality, implement proper recovery)
        if self.current_task:
            # Re-add the failed task with higher priority
            self.current_task.status = 'pending'
            self.add_task(self.current_task)
            self.current_task = None

        # Process next task
        self.process_next_task()

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()

    # Start processing tasks
    node.timer = node.create_timer(0.5, node.process_next_task)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down task planner node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [ROS 2 State Machines](https://github.com/ros-savvy/philosophy)
- [Task Planning in Robotics](https://arxiv.org/abs/1809.04615)
- [Integrated Robotics Systems](https://ieeexplore.ieee.org/document/8206289)
- [Multi-Modal Human-Robot Interaction](https://arxiv.org/abs/1903.09842)

## Quiz Questions

1. What are the key challenges in integrating multiple robotics subsystems?
2. How does state management help coordinate complex robotic behaviors?
3. What strategies can be used for error recovery in integrated systems?