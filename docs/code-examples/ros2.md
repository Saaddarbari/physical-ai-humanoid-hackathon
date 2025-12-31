---
sidebar_position: 2
---

# ROS 2 Examples: Communication and Architecture

This chapter provides comprehensive ROS 2 code examples covering communication patterns, node architecture, and best practices for robotics development.

## Learning Goals

- Implement ROS 2 nodes with proper lifecycle management
- Use different communication patterns (topics, services, actions)
- Apply parameter management and configuration
- Follow ROS 2 best practices and conventions

## Core Concepts

- **Node Lifecycle**: Proper initialization, execution, and cleanup
- **Communication Patterns**: Topics, services, and actions for different needs
- **Parameter Management**: Configurable values for flexible operation
- **Quality of Service**: Appropriate settings for different use cases

## Implementation Section

### Basic Publisher/Subscriber Example

```python
# basic_communication.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    # Create both publisher and subscriber in the same process
    publisher = MinimalPublisher()
    subscriber = MinimalSubscriber()

    try:
        # Spin both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)
        executor.spin()
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down nodes')
    finally:
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Parameter Management Example

```python
# parameter_example.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('message', 'Hello from parameter node!')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('topic_name', 'parameter_topic')

        # Get parameter values
        self.message = self.get_parameter('message').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.topic_name = self.get_parameter('topic_name').value

        # Create publisher with configurable topic
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)

        # Create timer with configurable rate
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.i = 0
        self.get_logger().info(
            f'Parameter node initialized with:\n'
            f'  message: {self.message}\n'
            f'  publish_rate: {self.publish_rate}\n'
            f'  topic_name: {self.topic_name}'
        )

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.message} ({self.i})'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()

    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        parameter_node.get_logger().info('Shutting down parameter node')
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Server and Client Example

```python
# service_example.py
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request received: {request.a} + {request.b} = {response.sum}'
        )
        return response

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.client.call_async(request)
        return self.future

def main_server(args=None):
    rclpy.init(args=args)
    server = AddTwoIntsServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down server')
    finally:
        server.destroy_node()
        rclpy.shutdown()

def main_client(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()

    if len(sys.argv) != 3:
        print('Usage: ros2 run package_name service_example_client -- 1 2')
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    future = client.send_request(a, b)

    try:
        rclpy.spin_until_future_complete(client, future)
        response = future.result()
        print(f'Result: {response.sum}')
    except KeyboardInterrupt:
        client.get_logger().info('Shutting down client')
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'client':
        main_client()
    else:
        main_server()
```

### Action Server and Client Example

```python
# action_example.py
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Send feedback periodically
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)

    action_server = FibonacciActionServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        action_server.get_logger().info('Shutting down action server')
    finally:
        action_server.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service Example

```python
# qos_example.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class QoSExampleNode(Node):
    def __init__(self):
        super().__init__('qos_example_node')

        # Different QoS profiles for different use cases
        # 1. Reliable communication (good for commands)
        cmd_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # 2. Best effort for sensor data (good for high-frequency data)
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        # 3. Keep all messages (good for logging)
        log_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL
        )

        # Create publishers with different QoS profiles
        self.cmd_publisher = self.create_publisher(String, 'cmd_topic', cmd_qos)
        self.sensor_publisher = self.create_publisher(String, 'sensor_topic', sensor_qos)
        self.log_publisher = self.create_publisher(String, 'log_topic', log_qos)

        # Create subscribers with matching QoS
        self.cmd_subscription = self.create_subscription(
            String, 'cmd_topic', self.cmd_callback, cmd_qos)
        self.sensor_subscription = self.create_subscription(
            String, 'sensor_topic', self.sensor_callback, sensor_qos)
        self.log_subscription = self.create_subscription(
            String, 'log_topic', self.log_callback, log_qos)

        # Timer for publishing messages
        self.timer = self.create_timer(1.0, self.publish_messages)

        self.get_logger().info('QoS example node started')

    def publish_messages(self):
        """Publish messages with different QoS requirements"""
        # Command message (reliable)
        cmd_msg = String()
        cmd_msg.data = 'Command message'
        self.cmd_publisher.publish(cmd_msg)

        # Sensor message (best effort)
        sensor_msg = String()
        sensor_msg.data = 'Sensor data'
        self.sensor_publisher.publish(sensor_msg)

        # Log message (keep all)
        log_msg = String()
        log_msg.data = f'Log message at {self.get_clock().now().nanoseconds}'
        self.log_publisher.publish(log_msg)

    def cmd_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')

    def sensor_callback(self, msg):
        self.get_logger().debug(f'Received sensor: {msg.data}')

    def log_callback(self, msg):
        self.get_logger().info(f'Received log: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = QoSExampleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down QoS example node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-settings.html)
- [ROS 2 Design Patterns](https://design.ros2.org/)

## Quiz Questions

1. What are the key differences between topics, services, and actions in ROS 2?
2. How do Quality of Service settings affect communication in ROS 2?
3. What are the best practices for parameter management in ROS 2 nodes?