---
sidebar_position: 1
---

# Voice Command Processing

This chapter covers the implementation of voice command processing systems for humanoid robots, enabling natural language interaction and control.

## Learning Goals

- Implement speech recognition for robot command interpretation
- Create natural language processing pipelines for command understanding
- Integrate voice commands with robot action execution
- Handle ambiguous or unclear voice commands gracefully

## Core Concepts

- **Speech Recognition**: Converting speech to text
- **Natural Language Processing**: Understanding command intent
- **Voice Command Mapping**: Connecting voice commands to robot actions
- **Error Handling**: Managing recognition errors and ambiguous commands

## Implementation Section

### Speech Recognition Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr
import threading
import queue

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_command_publisher = self.create_publisher(String, '/voice_command', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set up audio parameters
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)  # Adjust for background noise

        # Command mappings
        self.command_map = {
            'move forward': self.move_forward,
            'move backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'go forward': self.move_forward,
            'go back': self.move_backward,
            'spin left': self.turn_left,
            'spin right': self.turn_right,
        }

        # Start speech recognition in a separate thread
        self.audio_queue = queue.Queue()
        self.recognition_thread = threading.Thread(target=self.recognition_worker)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()

        # Start recognition
        self.start_listening()

        self.get_logger().info('Voice command node started')

    def start_listening(self):
        """Start listening for voice commands"""
        self.get_logger().info('Listening for voice commands...')

    def recognition_worker(self):
        """Worker thread for speech recognition"""
        try:
            with self.microphone as source:
                while rclpy.ok():
                    try:
                        # Listen for audio
                        audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)

                        # Add audio to queue for processing
                        self.audio_queue.put(audio)

                    except sr.WaitTimeoutError:
                        # Continue listening
                        continue
                    except Exception as e:
                        self.get_logger().error(f'Error in recognition: {e}')
                        continue
        except Exception as e:
            self.get_logger().error(f'Recognition worker error: {e}')

    def process_audio(self, audio):
        """Process audio and execute commands"""
        try:
            # Recognize speech using Google's speech recognition
            text = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f'Recognized: {text}')

            # Publish the recognized command
            cmd_msg = String()
            cmd_msg.data = text
            self.voice_command_publisher.publish(cmd_msg)

            # Execute command based on recognized text
            self.execute_command(text)

        except sr.UnknownValueError:
            self.get_logger().warn('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def execute_command(self, command_text):
        """Execute command based on recognized text"""
        # Try to match the command to our command map
        for keyword, action in self.command_map.items():
            if keyword in command_text:
                self.get_logger().info(f'Executing command: {keyword}')
                action()
                return

        # If no command matched, log it
        self.get_logger().info(f'Unrecognized command: {command_text}')

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.3  # Forward speed
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.3  # Backward speed
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Left turn speed
        self.cmd_vel_publisher.publish(cmd)

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.5  # Right turn speed
        self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        # Process audio from queue in main thread
        while rclpy.ok():
            try:
                # Check for audio in queue (non-blocking)
                if not node.audio_queue.empty():
                    audio = node.audio_queue.get_nowait()
                    node.process_audio(audio)
                else:
                    # Process ROS callbacks
                    rclpy.spin_once(node, timeout_sec=0.1)
            except queue.Empty:
                # Process ROS callbacks
                rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info('Shutting down voice command node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Natural Language Processing for Commands

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import re
import math

class NLUCommandProcessorNode(Node):
    def __init__(self):
        super().__init__('nlu_command_processor_node')

        # Create subscriptions
        self.voice_command_subscription = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )

        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize NLU patterns
        self.patterns = {
            'move_to': [
                r'move to (.+)',
                r'go to (.+)',
                r'go over to (.+)',
            ],
            'move_direction': [
                r'go (.+) by (.+)',
                r'move (.+) (.+)',
                r'(.+) (.+)',
            ],
            'stop': [
                r'stop',
                r'freeze',
                r'hold',
            ],
            'turn': [
                r'turn (.+)',
                r'spin (.+)',
                r'rotate (.+)',
            ]
        }

        # Location mappings (simplified)
        self.locations = {
            'kitchen': Point(x=3.0, y=1.0, z=0.0),
            'living room': Point(x=0.0, y=2.0, z=0.0),
            'bedroom': Point(x=-2.0, y=-1.0, z=0.0),
            'bathroom': Point(x=1.0, y=-2.0, z=0.0),
        }

        # Direction mappings
        self.directions = {
            'forward': (1, 0),
            'backward': (-1, 0),
            'back': (-1, 0),
            'left': (0, 1),
            'right': (0, -1),
        }

        # Turn directions
        self.turn_directions = {
            'left': math.pi / 2,
            'right': -math.pi / 2,
            'around': math.pi,
            'clockwise': -math.pi / 2,
            'counterclockwise': math.pi / 2,
        }

        self.get_logger().info('NLU command processor node started')

    def voice_command_callback(self, msg):
        """Process voice command using natural language understanding"""
        command = msg.data.lower()
        self.get_logger().info(f'Processing command: {command}')

        # Try to match command patterns
        processed = False

        # Check for location commands
        for location_name, location_point in self.locations.items():
            if location_name in command:
                self.get_logger().info(f'Navigating to {location_name}')
                self.navigate_to_location(location_point)
                processed = True
                break

        if not processed:
            # Check for directional commands
            for direction_name, (dx, dy) in self.directions.items():
                if direction_name in command:
                    # Extract distance if specified
                    distance = 1.0  # default distance
                    distance_match = re.search(r'(\d+\.?\d*)', command)
                    if distance_match:
                        distance = float(distance_match.group(1))

                    self.get_logger().info(f'Moving {direction_name} by {distance}m')
                    self.move_direction(dx, dy, distance)
                    processed = True
                    break

        if not processed:
            # Check for turn commands
            for turn_name, angle in self.turn_directions.items():
                if turn_name in command:
                    self.get_logger().info(f'Turning {turn_name}')
                    self.turn_by_angle(angle)
                    processed = True
                    break

        if not processed:
            # Check for stop commands
            if any(stop_word in command for stop_word in ['stop', 'freeze', 'hold']):
                self.get_logger().info('Stopping robot')
                self.stop_robot()
                processed = True

        if not processed:
            self.get_logger().info(f'Command not understood: {command}')

    def navigate_to_location(self, target_point):
        """Navigate to a specific location (simplified)"""
        # In a real implementation, this would use a navigation stack
        cmd = Twist()
        cmd.linear.x = 0.2  # Move forward slowly
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

        # This is a simplified implementation - real navigation would be more complex
        self.get_logger().info(f'Navigating to location: ({target_point.x}, {target_point.y})')

    def move_direction(self, dx, dy, distance):
        """Move in a specific direction for a given distance"""
        # Normalize direction vector
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            dx /= length
            dy /= length

        cmd = Twist()
        cmd.linear.x = 0.3 * dx
        cmd.linear.y = 0.3 * dy
        cmd.angular.z = 0.0  # No rotation while moving
        self.cmd_vel_publisher.publish(cmd)

        # This is simplified - in reality you'd track actual movement
        self.get_logger().info(f'Moving in direction ({dx}, {dy}) for {distance}m')

    def turn_by_angle(self, angle):
        """Turn by a specific angle"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5 if angle > 0 else -0.5  # Turn direction based on angle sign
        self.cmd_vel_publisher.publish(cmd)

        # This is simplified - real implementation would track actual rotation
        self.get_logger().info(f'Turning by {angle} radians')

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NLUCommandProcessorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NLU command processor node')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [Speech Recognition Library](https://pypi.org/project/SpeechRecognition/)
- [Natural Language Processing for Robotics](https://www.cs.cmu.edu/~./task20/publications/voice_control_robots.pdf)
- [ROS 2 Audio Processing](https://github.com/ros-speech/speech_recognition)
- [Voice Command Systems](https://arxiv.org/abs/2003.08516)

## Quiz Questions

1. What are the main challenges in implementing voice command processing for robots?
2. How does natural language understanding differ from simple keyword matching?
3. What error handling strategies are important for voice command systems?