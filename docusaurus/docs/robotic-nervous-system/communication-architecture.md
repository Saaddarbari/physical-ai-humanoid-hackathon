---
sidebar_position: 1
---

# Robot Communication Architecture

This chapter covers the fundamental communication patterns in ROS 2, including nodes, topics, services, and actions that form the backbone of robotic systems.

## Learning Goals

- Understand the ROS 2 communication architecture
- Implement nodes that communicate via topics
- Use services for request-response communication
- Apply actions for goal-oriented communication

## Core Concepts

- **Nodes**: Independent processes that perform computation in ROS
- **Topics**: Asynchronous communication channels for publishing and subscribing to messages
- **Services**: Synchronous request-response communication between nodes
- **Actions**: Asynchronous communication for long-running tasks with feedback

## Implementation Section

### Creating a Simple Publisher Node

```python
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
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Simple Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [ROS 2 Nodes Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Nodes.html)
- [ROS 2 Topics Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Topics.html)
- [ROS 2 Services Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Services.html)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Actions.html)

## Quiz Questions

1. What is the difference between a topic and a service in ROS 2?
2. When would you use an action instead of a service?
3. How do nodes discover each other in a ROS 2 network?