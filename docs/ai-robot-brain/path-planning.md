---
sidebar_position: 2
---

# Path Planning for Bipedal Humanoids

This chapter covers path planning algorithms specifically designed for bipedal humanoid robots, addressing the unique challenges of legged locomotion and balance.

## Learning Goals

- Understand the challenges of path planning for bipedal robots
- Implement footstep planning algorithms
- Apply balance-aware navigation techniques
- Consider terrain and stability constraints in planning

## Core Concepts

- **Footstep Planning**: Planning where and when to place feet
- **Stability Constraints**: Maintaining balance during locomotion
- **Terrain Analysis**: Understanding ground properties for safe walking
- **Dynamic Balance**: Maintaining center of mass during movement

## Implementation Section

### Simple Footstep Planner

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math

class FootstepPlannerNode(Node):
    def __init__(self):
        super().__init__('footstep_planner_node')

        # Create publisher for footstep markers
        self.footstep_pub = self.create_publisher(MarkerArray, 'footsteps', 10)

        # Create timer to plan footsteps
        self.timer = self.create_timer(1.0, self.plan_footsteps)

        # Robot parameters
        self.step_length = 0.3  # Distance between steps
        self.step_width = 0.2   # Distance between left and right feet
        self.step_height = 0.05 # Height of step motion

        # Target position
        self.target_x = 3.0
        self.target_y = 2.0

        # Current position (starting from origin)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.get_logger().info('Footstep planner node started')

    def plan_footsteps(self):
        # Calculate distance to target
        distance = math.sqrt((self.target_x - self.current_x)**2 + (self.target_y - self.current_y)**2)

        if distance < 0.1:  # Close enough to target
            return

        # Calculate direction to target
        target_yaw = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)

        # Determine number of steps needed
        num_steps = int(distance / self.step_length) + 1

        marker_array = MarkerArray()

        # Plan footsteps along the path
        for i in range(num_steps):
            # Calculate step position
            step_x = self.current_x + (i + 1) * self.step_length * math.cos(target_yaw)
            step_y = self.current_y + (i + 1) * self.step_length * math.sin(target_yaw)

            # Alternate between left and right foot
            foot_offset = self.step_width / 2.0 if i % 2 == 0 else -self.step_width / 2.0
            step_x_offset = step_x + foot_offset * math.sin(target_yaw)
            step_y_offset = step_y - foot_offset * math.cos(target_yaw)

            # Create marker for footstep
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "footsteps"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Set position
            marker.pose.position.x = step_x_offset
            marker.pose.position.y = step_y_offset
            marker.pose.position.z = 0.01  # Slightly above ground
            marker.pose.orientation.w = 1.0

            # Set scale (foot size)
            marker.scale.x = 0.15  # Foot length
            marker.scale.y = 0.1   # Foot width
            marker.scale.z = 0.01  # Foot height

            # Set color based on step number
            marker.color.r = float(i) / num_steps
            marker.color.g = 1.0 - float(i) / num_steps
            marker.color.b = 0.5
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        # Publish footsteps
        self.footstep_pub.publish(marker_array)

        self.get_logger().info(f'Planned {num_steps} footsteps to target')

def main(args=None):
    rclpy.init(args=args)
    node = FootstepPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down footstep planner node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Stability-Aware Path Planning

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
import numpy as np
from collections import deque

class StabilityPathPlannerNode(Node):
    def __init__(self):
        super().__init__('stability_path_planner_node')

        # Create subscription to occupancy grid
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Create publisher for planned path
        self.path_pub = self.create_publisher(MarkerArray, 'stability_path', 10)

        # Robot parameters
        self.robot_radius = 0.3  # Robot safety radius
        self.step_size = 0.2     # Size of each step in path planning
        self.max_slope = 0.3     # Maximum allowed slope for bipedal walking

        # Map data
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None

        self.get_logger().info('Stability-aware path planner node started')

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin

        # Plan path when new map is received
        self.plan_stable_path()

    def world_to_map(self, x, y):
        """Convert world coordinates to map indices"""
        if self.map_origin is None:
            return None, None

        map_x = int((x - self.map_origin.position.x) / self.map_resolution)
        map_y = int((y - self.map_origin.position.y) / self.map_resolution)

        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            return map_x, map_y
        else:
            return None, None

    def map_to_world(self, map_x, map_y):
        """Convert map indices to world coordinates"""
        if self.map_origin is None:
            return None, None

        world_x = map_x * self.map_resolution + self.map_origin.position.x
        world_y = map_y * self.map_resolution + self.map_origin.position.y

        return world_x, world_y

    def is_traversable(self, map_x, map_y):
        """Check if a cell is traversable for a bipedal robot"""
        if self.map_data is None or map_x < 0 or map_x >= self.map_width or map_y < 0 or map_y >= self.map_height:
            return False

        # Check if cell is free (value < 50 means not occupied)
        if self.map_data[map_y, map_x] >= 50:
            return False

        # Check surrounding cells for robot footprint
        robot_radius_cells = int(self.robot_radius / self.map_resolution)
        for dx in range(-robot_radius_cells, robot_radius_cells + 1):
            for dy in range(-robot_radius_cells, robot_radius_cells + 1):
                check_x = map_x + dx
                check_y = map_y + dy

                if 0 <= check_x < self.map_width and 0 <= check_y < self.map_height:
                    if self.map_data[check_y, check_x] >= 50:  # Occupied
                        return False

        return True

    def calculate_stability_cost(self, x1, y1, x2, y2):
        """Calculate stability cost based on terrain properties"""
        # For simplicity, we'll use distance as the primary cost
        # In a real implementation, this would consider terrain slope, roughness, etc.
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # Add penalty for cells near obstacles
        map_x2, map_y2 = self.world_to_map(x2, y2)
        if map_x2 is not None and map_y2 is not None:
            # Check for nearby obstacles
            penalty = 0
            robot_radius_cells = int(self.robot_radius / self.map_resolution)
            for dx in range(-robot_radius_cells, robot_radius_cells + 1):
                for dy in range(-robot_radius_cells, robot_radius_cells + 1):
                    check_x = map_x2 + dx
                    check_y = map_y2 + dy
                    if 0 <= check_x < self.map_width and 0 <= check_y < self.map_height:
                        if self.map_data[check_y, check_x] > 20:  # Near obstacle
                            penalty += 1

            return distance + penalty * 0.1
        else:
            return float('inf')

    def plan_stable_path(self):
        """Plan a path considering stability constraints"""
        if self.map_data is None:
            return

        # Define start and goal positions (these would normally come from a service call or parameter)
        start_x, start_y = 1.0, 1.0  # Starting position
        goal_x, goal_y = 8.0, 6.0    # Goal position

        start_map_x, start_map_y = self.world_to_map(start_x, start_y)
        goal_map_x, goal_map_y = self.world_to_map(goal_x, goal_y)

        if start_map_x is None or start_map_y is None or goal_map_x is None or goal_map_y is None:
            self.get_logger().warn('Start or goal position is outside map bounds')
            return

        if not self.is_traversable(start_map_x, start_map_y) or not self.is_traversable(goal_map_x, goal_map_y):
            self.get_logger().warn('Start or goal position is not traversable')
            return

        # Simple A* path planning (simplified for example)
        # In a real implementation, you would use a proper A* or D* algorithm
        path = self.a_star_search(start_map_x, start_map_y, goal_map_x, goal_map_y)

        if path:
            self.visualize_path(path)
        else:
            self.get_logger().warn('No path found')

    def a_star_search(self, start_x, start_y, goal_x, goal_y):
        """Simplified A* pathfinding algorithm"""
        # Use a priority queue for A* (simplified implementation)
        open_set = [(0, (start_x, start_y))]
        came_from = {}
        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.heuristic(start_x, start_y, goal_x, goal_y)}

        while open_set:
            # Simplified: just take the first element (in real A* you'd use a proper priority queue)
            current = min(open_set, key=lambda x: x[0])[1]
            open_set = [item for item in open_set if item[1] != current]

            if current == (goal_x, goal_y):
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((start_x, start_y))
                return path[::-1]  # Return reversed path

            # Check neighbors (8-connected)
            for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                # Check bounds
                if neighbor[0] < 0 or neighbor[0] >= self.map_width or neighbor[1] < 0 or neighbor[1] >= self.map_height:
                    continue

                # Check if traversable
                if not self.is_traversable(neighbor[0], neighbor[1]):
                    continue

                # Calculate tentative g_score
                movement_cost = math.sqrt(dx*dx + dy*dy) * self.map_resolution
                tentative_g_score = g_score[current] + movement_cost

                # If this path to neighbor is better than any previous one
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor[0], neighbor[1], goal_x, goal_y)

                    if neighbor not in [item[1] for item in open_set]:
                        open_set.append((f_score[neighbor], neighbor))

        return None  # No path found

    def heuristic(self, x1, y1, x2, y2):
        """Heuristic function for A* (Euclidean distance)"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2) * self.map_resolution

    def visualize_path(self, path):
        """Visualize the planned path"""
        marker_array = MarkerArray()

        for i, (map_x, map_y) in enumerate(path):
            world_x, world_y = self.map_to_world(map_x, map_y)
            if world_x is not None and world_y is not None:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "path"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = world_x
                marker.pose.position.y = world_y
                marker.pose.position.z = 0.1  # Slightly above ground
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)

        self.path_pub.publish(marker_array)
        self.get_logger().info(f'Planned path with {len(path)} waypoints')

def main(args=None):
    rclpy.init(args=args)
    node = StabilityPathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down stability path planner node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Additional Resources

- [Navigation2 for Legged Robots](https://navigation.ros.org/)
- [Footstep Planning Algorithms](https://ieeexplore.ieee.org/document/8206289)
- [Bipedal Locomotion Control](https://www.sciencedirect.com/science/article/pii/S0921889017304222)
- [Humanoid Robot Path Planning](https://link.springer.com/article/10.1007/s10514-019-09878-9)

## Quiz Questions

1. What are the main challenges of path planning for bipedal robots compared to wheeled robots?
2. How does footstep planning differ from traditional path planning?
3. What stability constraints must be considered when planning paths for humanoid robots?