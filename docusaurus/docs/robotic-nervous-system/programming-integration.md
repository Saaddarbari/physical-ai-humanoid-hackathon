---
sidebar_position: 2
---

# Programming Integration for Robotics

This chapter focuses on integrating Python programming with ROS 2 for robotics applications, emphasizing the rclpy library and best practices for robotics software development.

## Learning Goals

- Use the rclpy library for Python-based robotics development
- Implement proper node lifecycle management
- Handle parameters and configuration in ROS 2 nodes
- Apply error handling and logging in robotics applications

## Core Concepts

- **rclpy**: The Python client library for ROS 2
- **Node Lifecycle**: Proper initialization, execution, and cleanup of ROS nodes
- **Parameters**: Configurable values that can be set at runtime
- **Logging**: Proper logging practices for debugging and monitoring

## Implementation Section

### Creating a Parameterized Node

```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('message', 'Hello from parameter node!')
        self.declare_parameter('publish_rate', 1.0)

        # Get parameter values
        self.message = self.get_parameter('message').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'parameter_topic', 10)

        # Create timer with rate from parameter
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info(f'Parameter node initialized with message: {self.message}')

    def timer_callback(self):
        msg = String()
        msg.data = self.message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()

    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        parameter_node.get_logger().info('Interrupted by user')
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Error Handling and Logging Example

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from std_msgs.msg import String
import traceback

class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')

        try:
            # Declare and get parameters
            self.declare_parameter('topic_name', 'robust_topic')
            topic_name = self.get_parameter('topic_name').value

            # Create publisher
            self.publisher_ = self.create_publisher(String, topic_name, 10)

            # Create timer
            self.timer = self.create_timer(0.5, self.timer_callback)

            self.i = 0
            self.get_logger().info('Robust node initialized successfully')

        except ParameterNotDeclaredException as e:
            self.get_logger().error(f'Parameter not declared: {e}')
            raise
        except Exception as e:
            self.get_logger().error(f'Error during initialization: {e}')
            traceback.print_exc()
            raise

    def timer_callback(self):
        try:
            msg = String()
            msg.data = f'Robust message {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published message {self.i}')
            self.i += 1
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    robust_node = RobustNode()

    try:
        rclpy.spin(robust_node)
    except KeyboardInterrupt:
        robust_node.get_logger().info('Node interrupted by user')
    except Exception as e:
        robust_node.get_logger().error(f'Unhandled exception: {e}')
        traceback.print_exc()
    finally:
        robust_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [rclpy Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-CPP.html)
- [ROS 2 Logging Guide](https://docs.ros.org/en/humble/How-To-Guides/Logging.html)
- [Python Robotics Best Practices](https://pythonrobotics.github.io/)

## Quiz Questions

1. What are the advantages of using parameters in ROS 2 nodes?
2. How should you handle errors in ROS 2 node initialization?
3. What is the proper way to clean up resources in a ROS 2 node?