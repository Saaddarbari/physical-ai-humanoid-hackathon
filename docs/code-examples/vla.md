---
sidebar_position: 6
---

# Voice-Language-Action Examples: VLA Systems

This chapter provides comprehensive code examples for Voice-Language-Action (VLA) systems, covering speech recognition, natural language processing, and action planning for humanoid robots.

## Learning Goals

- Implement voice command processing systems
- Apply natural language understanding for robot commands
- Connect voice commands to robot actions
- Integrate VLA systems with perception and manipulation

## Core Concepts

- **Speech Recognition**: Converting speech to text
- **Natural Language Processing**: Understanding command intent
- **Action Mapping**: Connecting voice commands to robot actions
- **Command Sequencing**: Executing multi-step voice commands

## Implementation Section

### Voice Command Processing Node

```python
# voice_command_processor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from moveit_msgs.msg import MoveGroupAction
from actionlib_msgs.msg import GoalID
import speech_recognition as sr
import threading
import queue
import re
import time
import json
from typing import Dict, List, Optional

class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Create publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.voice_command_publisher = self.create_publisher(String, '/parsed_command', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set up audio parameters
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Command vocabulary and mappings
        self.command_vocabulary = {
            # Movement commands
            'move forward': 'MOVE_FORWARD',
            'move backward': 'MOVE_BACKWARD',
            'go forward': 'MOVE_FORWARD',
            'go back': 'MOVE_BACKWARD',
            'move up': 'MOVE_UP',
            'move down': 'MOVE_DOWN',
            'turn left': 'TURN_LEFT',
            'turn right': 'TURN_RIGHT',
            'spin left': 'SPIN_LEFT',
            'spin right': 'SPIN_RIGHT',
            'go left': 'STRAFE_LEFT',
            'go right': 'STRAFE_RIGHT',
            'stop': 'STOP',
            'halt': 'STOP',

            # Navigation commands
            'go to kitchen': 'NAVIGATE_TO_KITCHEN',
            'go to living room': 'NAVIGATE_TO_LIVING_ROOM',
            'go to bedroom': 'NAVIGATE_TO_BEDROOM',
            'go to bathroom': 'NAVIGATE_TO_BATHROOM',
            'go to office': 'NAVIGATE_TO_OFFICE',
            'go to dining room': 'NAVIGATE_TO_DINING_ROOM',

            # Object manipulation commands
            'pick up red ball': 'PICK_UP_RED_BALL',
            'pick up blue cube': 'PICK_UP_BLUE_CUBE',
            'pick up green cylinder': 'PICK_UP_GREEN_CYLINDER',
            'grasp object': 'GRASP_OBJECT',
            'take object': 'GRASP_OBJECT',
            'put down': 'PUT_DOWN',
            'release': 'RELEASE_GRIPPER',

            # Complex commands
            'bring me coffee': 'BRING_COFFEE',
            'get the red cup': 'GET_RED_CUP',
            'move the blue box': 'MOVE_BLUE_BOX',
            'clean the table': 'CLEAN_TABLE',
        }

        # Location mappings
        self.location_mappings = {
            'kitchen': (3.0, 1.0, 0.0),
            'living room': (0.0, 2.0, 0.0),
            'bedroom': (-2.0, -1.0, 0.0),
            'bathroom': (1.0, -2.0, 0.0),
            'office': (-1.0, 1.5, 0.0),
            'dining room': (2.5, -0.5, 0.0),
        }

        # Object mappings
        self.object_mappings = {
            'red ball': {'color': 'red', 'shape': 'ball', 'type': 'graspable'},
            'blue cube': {'color': 'blue', 'shape': 'cube', 'type': 'graspable'},
            'green cylinder': {'color': 'green', 'shape': 'cylinder', 'type': 'graspable'},
            'red cup': {'color': 'red', 'shape': 'cup', 'type': 'graspable'},
            'blue box': {'color': 'blue', 'shape': 'box', 'type': 'graspable'},
            'coffee': {'name': 'coffee', 'type': 'drinkable'},
        }

        # Audio queue for processing
        self.audio_queue = queue.Queue()
        self.listening_active = True

        # Start speech recognition thread
        self.recognition_thread = threading.Thread(target=self.recognition_worker)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()

        # Start recognition
        self.start_listening()

        self.get_logger().info('Voice command processor started')

    def start_listening(self):
        """Start listening for voice commands"""
        self.get_logger().info('Voice command processor is listening...')

    def recognition_worker(self):
        """Background thread for continuous speech recognition"""
        try:
            with self.microphone as source:
                while self.listening_active and rclpy.ok():
                    try:
                        # Listen for audio
                        audio = self.recognizer.listen(source, timeout=0.5, phrase_time_limit=5)

                        # Add audio to queue for processing
                        self.audio_queue.put(audio)

                    except sr.WaitTimeoutError:
                        # Continue listening
                        continue
                    except Exception as e:
                        self.get_logger().error(f'Error in recognition: {e}')
                        time.sleep(0.1)  # Brief pause before retrying
                        continue
        except Exception as e:
            self.get_logger().error(f'Recognition worker error: {e}')

    def process_audio(self, audio):
        """Process audio and execute commands"""
        try:
            # Recognize speech using Google's speech recognition
            text = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f'Recognized: {text}')

            # Publish raw recognized text
            raw_msg = String()
            raw_msg.data = text
            self.voice_command_publisher.publish(raw_msg)

            # Parse and execute command
            self.parse_and_execute_command(text)

        except sr.UnknownValueError:
            self.get_logger().warn('Could not understand audio')
        except sr.RequestError as e:
            self.get_logger().error(f'Speech recognition error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def parse_and_execute_command(self, command_text: str):
        """Parse command text and execute appropriate action"""
        # First, try exact matches
        if command_text in self.command_vocabulary:
            command_type = self.command_vocabulary[command_text]
            self.execute_command(command_type)
            return

        # If no exact match, use pattern matching
        parsed_command = self.parse_command_semantics(command_text)
        if parsed_command:
            self.execute_parsed_command(parsed_command)

    def parse_command_semantics(self, command_text: str) -> Optional[Dict]:
        """Parse command semantics using regex patterns"""
        command_lower = command_text.lower()

        # Pattern: "go to [location]"
        go_to_pattern = r'go to (\w+(?:\s+\w+)*)'
        go_to_match = re.search(go_to_pattern, command_lower)
        if go_to_match:
            location = go_to_match.group(1).strip()
            if location in self.location_mappings:
                return {
                    'type': 'NAVIGATE',
                    'location': location,
                    'coordinates': self.location_mappings[location]
                }

        # Pattern: "pick up [object]" or "grasp [object]"
        pick_up_pattern = r'(?:pick up|grasp|take|get) (\w+(?:\s+\w+)*)'
        pick_up_match = re.search(pick_up_pattern, command_lower)
        if pick_up_match:
            obj = pick_up_match.group(1).strip()
            if obj in self.object_mappings:
                return {
                    'type': 'PICK_UP',
                    'object': obj,
                    'properties': self.object_mappings[obj]
                }

        # Pattern: "move [object]"
        move_obj_pattern = r'move (\w+(?:\s+\w+)*)'
        move_obj_match = re.search(move_obj_pattern, command_lower)
        if move_obj_match:
            obj = move_obj_match.group(1).strip()
            if obj in self.object_mappings:
                return {
                    'type': 'MOVE_OBJECT',
                    'object': obj,
                    'properties': self.object_mappings[obj]
                }

        # Simple movement commands
        movement_keywords = {
            'forward': 'MOVE_FORWARD',
            'back': 'MOVE_BACKWARD',
            'backward': 'MOVE_BACKWARD',
            'left': 'TURN_LEFT',
            'right': 'TURN_RIGHT',
            'up': 'MOVE_UP',
            'down': 'MOVE_DOWN'
        }

        for keyword, command_type in movement_keywords.items():
            if keyword in command_lower:
                return {
                    'type': command_type,
                    'direction': keyword
                }

        return None

    def execute_parsed_command(self, parsed_command: Dict):
        """Execute a parsed command"""
        cmd_type = parsed_command['type']
        self.get_logger().info(f'Executing parsed command: {parsed_command}')

        if cmd_type == 'NAVIGATE':
            self.navigate_to_location(parsed_command['coordinates'])
        elif cmd_type == 'PICK_UP':
            self.pick_up_object(parsed_command['object'])
        elif cmd_type == 'MOVE_OBJECT':
            self.move_object(parsed_command['object'])
        elif cmd_type in ['MOVE_FORWARD', 'MOVE_BACKWARD', 'TURN_LEFT', 'TURN_RIGHT', 'MOVE_UP', 'MOVE_DOWN']:
            self.execute_movement_command(cmd_type)
        else:
            self.get_logger().warn(f'Unknown command type: {cmd_type}')

    def execute_command(self, command_type: str):
        """Execute a recognized command"""
        self.get_logger().info(f'Executing command: {command_type}')

        command_functions = {
            'MOVE_FORWARD': self.move_forward,
            'MOVE_BACKWARD': self.move_backward,
            'TURN_LEFT': self.turn_left,
            'TURN_RIGHT': self.turn_right,
            'SPIN_LEFT': self.spin_left,
            'SPIN_RIGHT': self.spin_right,
            'STRAFE_LEFT': self.strafe_left,
            'STRAFE_RIGHT': self.strafe_right,
            'STOP': self.stop_robot,
            'NAVIGATE_TO_KITCHEN': lambda: self.navigate_to_location((3.0, 1.0, 0.0)),
            'NAVIGATE_TO_LIVING_ROOM': lambda: self.navigate_to_location((0.0, 2.0, 0.0)),
            'NAVIGATE_TO_BEDROOM': lambda: self.navigate_to_location((-2.0, -1.0, 0.0)),
            'NAVIGATE_TO_BATHROOM': lambda: self.navigate_to_location((1.0, -2.0, 0.0)),
            'PICK_UP_RED_BALL': lambda: self.pick_up_object('red ball'),
            'PICK_UP_BLUE_CUBE': lambda: self.pick_up_object('blue cube'),
            'PICK_UP_GREEN_CYLINDER': lambda: self.pick_up_object('green cylinder'),
            'GRASP_OBJECT': self.grasp_object,
            'PUT_DOWN': self.put_down_object,
            'RELEASE_GRIPPER': self.release_gripper,
        }

        if command_type in command_functions:
            command_functions[command_type]()
        else:
            self.get_logger().warn(f'Command not implemented: {command_type}')

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Moving forward')

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.3
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Moving backward')

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Turning left')

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.5
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Turning right')

    def spin_left(self):
        """Spin robot left in place"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 1.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Spinning left')

    def spin_right(self):
        """Spin robot right in place"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -1.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Spinning right')

    def strafe_left(self):
        """Strafe robot left"""
        cmd = Twist()
        cmd.linear.y = 0.3
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Strafing left')

    def strafe_right(self):
        """Strafe robot right"""
        cmd = Twist()
        cmd.linear.y = -0.3
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        self.get_logger().info('Strafing right')

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
        self.get_logger().info('Robot stopped')

    def navigate_to_location(self, coordinates: Tuple[float, float, float]):
        """Navigate to a specific location (simplified for example)"""
        target_x, target_y, target_theta = coordinates
        self.get_logger().info(f'Navigating to location: ({target_x}, {target_y})')

        # In a real system, this would call navigation services
        # For this example, we'll just log the intention
        self.get_logger().info(f'Navigation to ({target_x}, {target_y}) initiated')

    def pick_up_object(self, object_name: str):
        """Pick up a specific object"""
        self.get_logger().info(f'Attempting to pick up {object_name}')

        # In a real system, this would involve:
        # 1. Perceiving the object location
        # 2. Planning approach trajectory
        # 3. Executing grasp
        # For this example, we'll just log the action
        self.get_logger().info(f'Pick up operation for {object_name} initiated')

    def move_object(self, object_name: str):
        """Move an object to a new location"""
        self.get_logger().info(f'Moving {object_name} to new location')

        # In a real system, this would involve picking up and placing
        self.get_logger().info(f'Move object operation for {object_name} initiated')

    def grasp_object(self):
        """Grasp the nearest object"""
        self.get_logger().info('Attempting to grasp nearest object')

    def put_down_object(self):
        """Put down currently held object"""
        self.get_logger().info('Putting down object')

    def release_gripper(self):
        """Release gripper"""
        self.get_logger().info('Releasing gripper')

    def execute_movement_command(self, command_type: str):
        """Execute movement commands from parsed semantics"""
        movement_commands = {
            'MOVE_FORWARD': self.move_forward,
            'MOVE_BACKWARD': self.move_backward,
            'TURN_LEFT': self.turn_left,
            'TURN_RIGHT': self.turn_right,
            'MOVE_UP': self.move_forward,  # Map to forward for ground robot
            'MOVE_DOWN': self.move_backward,  # Map to backward for ground robot
        }

        if command_type in movement_commands:
            movement_commands[command_type]()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandProcessor()

    try:
        # Main loop to process audio from queue
        while rclpy.ok():
            try:
                # Check for audio in queue (non-blocking)
                if not node.audio_queue.empty():
                    audio = node.audio_queue.get_nowait()
                    node.process_audio(audio)
                else:
                    # Process ROS callbacks
                    rclpy.spin_once(node, timeout_sec=0.01)
            except queue.Empty:
                # Process ROS callbacks
                rclpy.spin_once(node, timeout_sec=0.01)
            except KeyboardInterrupt:
                node.get_logger().info('Shutting down voice command processor')
                break

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Natural Language Understanding for VLA

```python
# nlu_processor.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import re
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class ParsedCommand:
    """Structured representation of a parsed command"""
    intent: str
    entities: Dict[str, str]
    confidence: float
    original_text: str

class NaturalLanguageUnderstanding(Node):
    def __init__(self):
        super().__init__('natural_language_understanding')

        # Create subscriptions
        self.voice_command_subscription = self.create_subscription(
            String,
            '/recognized_speech',
            self.voice_command_callback,
            10
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create publishers
        self.parsed_command_publisher = self.create_publisher(String, '/parsed_command', 10)
        self.debug_publisher = self.create_publisher(String, '/nlu_debug', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Intent patterns and templates
        self.intent_patterns = {
            'navigation': [
                r'go to (\w+(?:\s+\w+)*)',
                r'go to the (\w+(?:\s+\w+)*)',
                r'walk to (\w+(?:\s+\w+)*)',
                r'move to (\w+(?:\s+\w+)*)',
                r'take me to (\w+(?:\s+\w+)*)',
                r'navigate to (\w+(?:\s+\w+)*)',
            ],
            'object_interaction': [
                r'(?:pick up|grasp|take|get) (\w+(?:\s+\w+)*)',
                r'(?:pick up|grasp|take|get) the (\w+(?:\s+\w+)*)',
                r'(?:pick up|grasp|take|get) that (\w+(?:\s+\w+)*)',
                r'bring me (\w+(?:\s+\w+)*)',
                r'hand me (\w+(?:\s+\w+)*)',
                r'give me (\w+(?:\s+\w+)*)',
            ],
            'movement': [
                r'(?:move|go|drive) (forward|backward|back|left|right|up|down)',
                r'(?:turn|rotate) (left|right)',
                r'spin (left|right)',
                r'strafe (left|right)',
                r'please (stop|halt)',
            ],
            'action_sequence': [
                r'(?:first|then|after that|next) (.+)',
                r'do this then (.+)',
                r'after that (.+)',
            ]
        }

        # Entity extraction patterns
        self.entity_patterns = {
            'location': [
                'kitchen', 'living room', 'bedroom', 'bathroom', 'office', 'dining room',
                'hallway', 'garage', 'garden', 'patio', 'entrance', 'exit'
            ],
            'object': [
                'ball', 'cube', 'cylinder', 'cup', 'box', 'book', 'bottle',
                'chair', 'table', 'couch', 'lamp', 'plant', 'door', 'window'
            ],
            'color': [
                'red', 'blue', 'green', 'yellow', 'orange', 'purple',
                'pink', 'brown', 'black', 'white', 'gray', 'grey'
            ],
            'size': [
                'small', 'large', 'big', 'tiny', 'huge', 'little'
            ]
        }

        # Context for multi-turn conversations
        self.context = {
            'last_command': None,
            'last_object': None,
            'last_location': None,
            'current_room': 'unknown'
        }

        self.get_logger().info('Natural Language Understanding node started')

    def voice_command_callback(self, msg: String):
        """Process voice command using NLU"""
        command_text = msg.data.lower()
        self.get_logger().info(f'Processing command: "{command_text}"')

        # Parse the command
        parsed_command = self.parse_command(command_text)

        if parsed_command:
            self.get_logger().info(f'Parsed command: {parsed_command.intent} with entities {parsed_command.entities}')

            # Publish parsed command
            parsed_msg = String()
            parsed_msg.data = json.dumps({
                'intent': parsed_command.intent,
                'entities': parsed_command.entities,
                'confidence': parsed_command.confidence,
                'original': parsed_command.original_text
            })
            self.parsed_command_publisher.publish(parsed_msg)

            # Execute action based on parsed command
            self.execute_action(parsed_command)

            # Update context
            self.update_context(parsed_command)
        else:
            self.get_logger().warn(f'Could not parse command: {command_text}')
            # Try to handle with fuzzy matching
            self.fuzzy_command_matching(command_text)

    def parse_command(self, command_text: str) -> Optional[ParsedCommand]:
        """Parse command using pattern matching and semantic analysis"""
        # Clean command text
        cleaned_text = re.sub(r'[^\w\s]', ' ', command_text.lower())
        cleaned_text = ' '.join(cleaned_text.split())  # Normalize whitespace

        # Try to match each intent pattern
        for intent, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, cleaned_text)
                if match:
                    entities = self.extract_entities(cleaned_text, match.groups())
                    confidence = self.calculate_confidence(intent, match, cleaned_text)

                    return ParsedCommand(
                        intent=intent,
                        entities=entities,
                        confidence=confidence,
                        original_text=command_text
                    )

        # If no pattern matches, try semantic similarity
        return self.semantic_similarity_parse(command_text)

    def extract_entities(self, text: str, matched_groups: Tuple) -> Dict[str, str]:
        """Extract entities from matched text"""
        entities = {}

        # Add matched groups as entities
        for i, group in enumerate(matched_groups):
            if group:
                # Try to classify the entity
                classified_entity = self.classify_entity(group.strip())
                if classified_entity:
                    entity_type, entity_value = classified_entity
                    entities[entity_type] = entity_value

        # Extract additional entities from the full text
        for entity_type, keywords in self.entity_patterns.items():
            for keyword in keywords:
                if keyword in text and entity_type not in entities:
                    entities[entity_type] = keyword

        return entities

    def classify_entity(self, entity_text: str) -> Optional[Tuple[str, str]]:
        """Classify an entity into its type"""
        entity_lower = entity_text.lower()

        # Check each entity type
        for entity_type, keywords in self.entity_patterns.items():
            for keyword in keywords:
                if keyword in entity_lower:
                    return entity_type, entity_text

        # If no specific classification, return as generic
        return 'generic', entity_text

    def calculate_confidence(self, intent: str, match, text: str) -> float:
        """Calculate confidence score for the parsed command"""
        # Base confidence on pattern match quality
        base_confidence = 0.8

        # Adjust based on text length and match position
        match_length = len(match.group(0))
        text_length = len(text)
        length_ratio = match_length / text_length if text_length > 0 else 0

        # Adjust confidence based on context
        context_adjustment = 0
        if self.context['current_room'] in text:
            context_adjustment += 0.1

        confidence = base_confidence * (0.5 + 0.5 * length_ratio) + context_adjustment
        return min(1.0, max(0.0, confidence))

    def semantic_similarity_parse(self, command_text: str) -> Optional[ParsedCommand]:
        """Use semantic similarity to parse commands that don't match patterns"""
        # This is a simplified version - in practice, you'd use embeddings or more sophisticated NLP
        command_lower = command_text.lower()

        # Define semantic categories
        semantic_categories = {
            'navigation': ['go', 'move', 'walk', 'drive', 'navigate', 'to', 'toward'],
            'object_interaction': ['pick', 'grasp', 'take', 'get', 'bring', 'hand', 'give'],
            'movement': ['forward', 'backward', 'back', 'left', 'right', 'turn', 'rotate', 'stop'],
            'action_sequence': ['first', 'then', 'next', 'after', 'when']
        }

        # Calculate semantic similarity
        best_intent = None
        best_score = 0

        for intent, keywords in semantic_categories.items():
            score = sum(1 for keyword in keywords if keyword in command_lower)
            if score > best_score:
                best_score = score
                best_intent = intent

        if best_intent and best_score > 0:
            # Extract entities based on semantic analysis
            entities = self.extract_entities(command_text, tuple())
            confidence = min(0.7, best_score / len(command_lower.split()))

            return ParsedCommand(
                intent=best_intent,
                entities=entities,
                confidence=confidence,
                original_text=command_text
            )

        return None

    def fuzzy_command_matching(self, command_text: str):
        """Handle commands that don't match any patterns using fuzzy matching"""
        self.get_logger().info(f'Using fuzzy matching for: {command_text}')

        # In a real system, this would use fuzzy string matching
        # or more sophisticated NLP techniques
        fuzzy_responses = [
            "I'm not sure what you mean. Could you rephrase that?",
            "I didn't quite catch that. Could you say it again?",
            "I'm still learning. Could you use simpler words?"
        ]

        import random
        response = random.choice(fuzzy_responses)
        self.get_logger().info(f'Fuzzy response: {response}')

    def execute_action(self, parsed_command: ParsedCommand):
        """Execute action based on parsed command"""
        intent = parsed_command.intent
        entities = parsed_command.entities

        self.get_logger().info(f'Executing action for intent: {intent}')

        action_functions = {
            'navigation': self.execute_navigation,
            'object_interaction': self.execute_object_interaction,
            'movement': self.execute_movement,
            'action_sequence': self.execute_action_sequence
        }

        if intent in action_functions:
            action_functions[intent](entities)
        else:
            self.get_logger().warn(f'Unknown intent: {intent}')

    def execute_navigation(self, entities: Dict[str, str]):
        """Execute navigation action"""
        if 'location' in entities:
            location = entities['location']
            self.get_logger().info(f'Navigating to {location}')
            # In a real system, this would call navigation services
        else:
            self.get_logger().warn('No location specified for navigation')

    def execute_object_interaction(self, entities: Dict[str, str]):
        """Execute object interaction action"""
        if 'object' in entities:
            obj = entities['object']
            self.get_logger().info(f'Interacting with {obj}')
            # In a real system, this would call manipulation services
        else:
            self.get_logger().warn('No object specified for interaction')

    def execute_movement(self, entities: Dict[str, str]):
        """Execute movement action"""
        if 'direction' in entities:
            direction = entities['direction']
            self.get_logger().info(f'Moving {direction}')
            # In a real system, this would send movement commands
        else:
            self.get_logger().warn('No direction specified for movement')

    def execute_action_sequence(self, entities: Dict[str, str]):
        """Execute sequence of actions"""
        self.get_logger().info('Executing action sequence')
        # In a real system, this would manage multi-step tasks

    def update_context(self, parsed_command: ParsedCommand):
        """Update conversation context"""
        entities = parsed_command.entities

        if 'object' in entities:
            self.context['last_object'] = entities['object']
        if 'location' in entities:
            self.context['last_location'] = entities['location']

        self.context['last_command'] = parsed_command.intent

    def image_callback(self, msg: Image):
        """Process camera image for visual context"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # In a real system, this would perform object detection
            # and update the visual context
            self.get_logger().debug('Processed camera image for visual context')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = NaturalLanguageUnderstanding()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down NLU node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### VLA Action Planning and Execution

```python
# vla_action_planner.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan, Image
from moveit_msgs.msg import MoveGroupAction, ExecuteTrajectoryGoal
from actionlib_msgs.msg import GoalStatus
from typing import List, Dict, Any, Optional
import time
import threading
from dataclasses import dataclass
from enum import Enum

class TaskStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass
class Task:
    """Represents a single task in the VLA system"""
    id: str
    intent: str
    entities: Dict[str, str]
    priority: int = 1
    dependencies: List[str] = None
    status: TaskStatus = TaskStatus.PENDING

class VLAActionPlanner(Node):
    def __init__(self):
        super().__init__('vla_action_planner')

        # Create subscriptions
        self.parsed_command_subscription = self.create_subscription(
            String,
            '/parsed_command',
            self.parsed_command_callback,
            10
        )

        self.task_status_subscription = self.create_subscription(
            String,
            '/task_status',
            self.task_status_callback,
            10
        )

        # Create publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_queue_publisher = self.create_publisher(String, '/current_task', 10)
        self.action_status_publisher = self.create_publisher(String, '/action_status', 10)

        # Task management
        self.task_queue = []  # Priority queue
        self.active_tasks = {}  # Currently executing tasks
        self.completed_tasks = []  # History of completed tasks
        self.failed_tasks = []  # History of failed tasks

        # Robot state
        self.current_pose = Pose()
        self.current_task = None
        self.is_executing = False

        # Create timer for task execution
        self.execution_timer = self.create_timer(0.1, self.execute_next_task)

        self.get_logger().info('VLA Action Planner started')

    def parsed_command_callback(self, msg: String):
        """Process parsed command and create tasks"""
        try:
            import json
            command_data = json.loads(msg.data)

            intent = command_data['intent']
            entities = command_data['entities']
            original = command_data['original']

            self.get_logger().info(f'Received command: {intent} with entities {entities}')

            # Create tasks based on intent
            tasks = self.create_tasks_from_intent(intent, entities, original)

            # Add tasks to queue
            for task in tasks:
                self.add_task(task)

        except Exception as e:
            self.get_logger().error(f'Error processing parsed command: {e}')

    def create_tasks_from_intent(self, intent: str, entities: Dict[str, str], original: str) -> List[Task]:
        """Create tasks based on intent and entities"""
        tasks = []

        if intent == 'navigation':
            # Navigation task
            task = Task(
                id=f"nav_{int(time.time())}",
                intent=intent,
                entities=entities,
                priority=2
            )
            tasks.append(task)

        elif intent == 'object_interaction':
            # Sequence of tasks: navigate to object, detect object, grasp object
            nav_task = Task(
                id=f"nav_obj_{int(time.time())}",
                intent='navigate_to_object',
                entities=entities,
                priority=3
            )

            detect_task = Task(
                id=f"detect_obj_{int(time.time())}",
                intent='detect_object',
                entities=entities,
                priority=3,
                dependencies=[nav_task.id]
            )

            grasp_task = Task(
                id=f"grasp_obj_{int(time.time())}",
                intent='grasp_object',
                entities=entities,
                priority=3,
                dependencies=[detect_task.id]
            )

            tasks.extend([nav_task, detect_task, grasp_task])

        elif intent == 'movement':
            # Simple movement task
            task = Task(
                id=f"move_{int(time.time())}",
                intent=intent,
                entities=entities,
                priority=1
            )
            tasks.append(task)

        else:
            # Default task for unrecognized intents
            task = Task(
                id=f"default_{int(time.time())}",
                intent=intent,
                entities=entities,
                priority=1
            )
            tasks.append(task)

        return tasks

    def add_task(self, task: Task):
        """Add a task to the queue"""
        # Insert based on priority (higher priority first)
        inserted = False
        for i, existing_task in enumerate(self.task_queue):
            if task.priority > existing_task.priority:
                self.task_queue.insert(i, task)
                inserted = True
                break

        if not inserted:
            self.task_queue.append(task)

        self.get_logger().info(f'Added task {task.id} with intent {task.intent} (priority: {task.priority})')

    def execute_next_task(self):
        """Execute the next task in the queue"""
        if self.is_executing or not self.task_queue:
            return

        # Check dependencies before executing
        ready_tasks = []
        for task in self.task_queue:
            if self.dependencies_satisfied(task):
                ready_tasks.append(task)

        if not ready_tasks:
            return  # No tasks ready to execute

        # Take the highest priority ready task
        task_to_execute = ready_tasks[0]

        # Remove from queue and add to active tasks
        self.task_queue.remove(task_to_execute)
        self.active_tasks[task_to_execute.id] = task_to_execute

        # Execute the task
        self.current_task = task_to_execute
        self.is_executing = True
        task_to_execute.status = TaskStatus.EXECUTING

        self.get_logger().info(f'Executing task {task_to_execute.id}: {task_to_execute.intent}')

        # Publish current task
        task_msg = String()
        task_msg.data = f"{task_to_execute.intent}:{task_to_execute.id}"
        self.task_queue_publisher.publish(task_msg)

        # Execute the specific action
        self.execute_task_action(task_to_execute)

    def dependencies_satisfied(self, task: Task) -> bool:
        """Check if task dependencies are satisfied"""
        if not task.dependencies:
            return True

        for dep_id in task.dependencies:
            # Check if dependency was completed successfully
            for completed_task in self.completed_tasks:
                if completed_task.id == dep_id:
                    return True

        return False

    def execute_task_action(self, task: Task):
        """Execute the specific action for a task"""
        action_methods = {
            'navigation': self.execute_navigation_task,
            'navigate_to_object': self.execute_navigate_to_object_task,
            'detect_object': self.execute_detect_object_task,
            'grasp_object': self.execute_grasp_object_task,
            'movement': self.execute_movement_task,
        }

        intent = task.intent
        if intent in action_methods:
            try:
                # Execute in a separate thread to avoid blocking
                thread = threading.Thread(target=action_methods[intent], args=(task,))
                thread.daemon = True
                thread.start()
            except Exception as e:
                self.get_logger().error(f'Error executing task {task.id}: {e}')
                self.complete_task(task.id, success=False)
        else:
            self.get_logger().warn(f'Unknown task intent: {intent}')
            self.complete_task(task.id, success=False)

    def execute_navigation_task(self, task: Task):
        """Execute navigation task"""
        entities = task.entities
        if 'location' in entities:
            location = entities['location']
            self.get_logger().info(f'Navigating to {location}')

            # Simulate navigation (in real system, this would call navigation stack)
            # For simulation, we'll just move forward for a bit
            cmd = Twist()
            cmd.linear.x = 0.3  # Move forward
            cmd.angular.z = 0.0

            # Simulate navigation time
            start_time = time.time()
            while time.time() - start_time < 3.0 and rclpy.ok():  # Navigate for 3 seconds
                self.cmd_vel_publisher.publish(cmd)
                time.sleep(0.1)

            # Stop robot
            cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(cmd)

            self.get_logger().info(f'Navigation to {location} completed')
            self.complete_task(task.id, success=True)
        else:
            self.get_logger().warn('No location specified for navigation')
            self.complete_task(task.id, success=False)

    def execute_navigate_to_object_task(self, task: Task):
        """Execute navigate to object task"""
        entities = task.entities
        if 'object' in entities:
            obj = entities['object']
            self.get_logger().info(f'Navigating to {obj}')

            # Simulate navigation to object
            cmd = Twist()
            cmd.linear.x = 0.2  # Slow approach
            cmd.angular.z = 0.0

            # Simulate navigation time
            start_time = time.time()
            while time.time() - start_time < 2.0 and rclpy.ok():
                self.cmd_vel_publisher.publish(cmd)
                time.sleep(0.1)

            cmd.linear.x = 0.0
            self.cmd_vel_publisher.publish(cmd)

            self.get_logger().info(f'Navigation to {obj} completed')
            self.complete_task(task.id, success=True)
        else:
            self.get_logger().warn('No object specified for navigation')
            self.complete_task(task.id, success=False)

    def execute_detect_object_task(self, task: Task):
        """Execute object detection task"""
        entities = task.entities
        if 'object' in entities:
            obj = entities['object']
            self.get_logger().info(f'Detecting {obj}')

            # Simulate object detection (in real system, this would process camera data)
            time.sleep(1.0)  # Simulate detection time

            # Simulate detection result
            import random
            detected = random.random() > 0.2  # 80% success rate

            if detected:
                self.get_logger().info(f'{obj} detected successfully')
                self.complete_task(task.id, success=True)
            else:
                self.get_logger().warn(f'{obj} not found')
                self.complete_task(task.id, success=False)
        else:
            self.get_logger().warn('No object specified for detection')
            self.complete_task(task.id, success=False)

    def execute_grasp_object_task(self, task: Task):
        """Execute object grasping task"""
        entities = task.entities
        if 'object' in entities:
            obj = entities['object']
            self.get_logger().info(f'Attempting to grasp {obj}')

            # Simulate grasping action (in real system, this would control manipulator)
            time.sleep(2.0)  # Simulate grasping time

            # Simulate grasping result
            import random
            success = random.random() > 0.3  # 70% success rate

            if success:
                self.get_logger().info(f'{obj} grasped successfully')
                self.complete_task(task.id, success=True)
            else:
                self.get_logger().warn(f'Grasping {obj} failed')
                self.complete_task(task.id, success=False)
        else:
            self.get_logger().warn('No object specified for grasping')
            self.complete_task(task.id, success=False)

    def execute_movement_task(self, task: Task):
        """Execute movement task"""
        entities = task.entities
        if 'direction' in entities:
            direction = entities['direction']
            self.get_logger().info(f'Executing movement: {direction}')

            cmd = Twist()

            # Map direction to movement
            if direction in ['forward', 'up']:
                cmd.linear.x = 0.3
            elif direction in ['backward', 'back', 'down']:
                cmd.linear.x = -0.3
            elif direction == 'left':
                cmd.angular.z = 0.5
            elif direction == 'right':
                cmd.angular.z = -0.5
            elif direction == 'stop':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            # Execute movement for 1 second
            start_time = time.time()
            while time.time() - start_time < 1.0 and rclpy.ok():
                self.cmd_vel_publisher.publish(cmd)
                time.sleep(0.1)

            # Stop robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)

            self.get_logger().info(f'Movement {direction} completed')
            self.complete_task(task.id, success=True)
        else:
            self.get_logger().warn('No direction specified for movement')
            self.complete_task(task.id, success=False)

    def complete_task(self, task_id: str, success: bool = True):
        """Mark a task as completed"""
        if task_id in self.active_tasks:
            task = self.active_tasks[task_id]
            task.status = TaskStatus.COMPLETED if success else TaskStatus.FAILED

            if success:
                self.completed_tasks.append(task)
            else:
                self.failed_tasks.append(task)

            del self.active_tasks[task_id]
            self.is_executing = False

            # Publish task completion status
            status_msg = String()
            status_msg.data = f"task_completed:{task_id}:{'success' if success else 'failure'}"
            self.action_status_publisher.publish(status_msg)

            self.get_logger().info(f'Task {task_id} completed with status: {"success" if success else "failure"}')

    def task_status_callback(self, msg: String):
        """Handle external task status updates"""
        # In a real system, this would handle status updates from other nodes
        status_parts = msg.data.split(':')
        if len(status_parts) >= 2:
            status_type, task_id = status_parts[0], status_parts[1]
            self.get_logger().info(f'Received external status update: {status_type} for task {task_id}')

def main(args=None):
    rclpy.init(args=args)
    node = VLAActionPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VLA Action Planner')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [Speech Recognition in Python](https://pypi.org/project/SpeechRecognition/)
- [Natural Language Processing for Robotics](https://www.cs.cmu.edu/~./task20/publications/voice_control_robots.pdf)
- [Voice Command Systems](https://arxiv.org/abs/2003.08516)
- [Robot Command Understanding](https://ieeexplore.ieee.org/document/8206289)

## Quiz Questions

1. What are the main components of a Voice-Language-Action system?
2. How does natural language understanding bridge voice commands to robot actions?
3. What challenges arise when implementing VLA systems for humanoid robots?