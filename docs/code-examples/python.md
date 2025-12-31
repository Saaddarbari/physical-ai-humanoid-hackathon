---
sidebar_position: 5
---

# Python Robotics Examples: General Programming Patterns

This chapter provides comprehensive Python code examples covering general robotics programming patterns, mathematical foundations, and common algorithms used in robotics applications.

## Learning Goals

- Implement core robotics algorithms in Python
- Apply mathematical foundations for robotics (kinematics, dynamics)
- Use Python libraries for robotics development
- Understand common robotics programming patterns

## Core Concepts

- **Kinematics**: Forward and inverse kinematics calculations
- **Trajectory Planning**: Path generation and interpolation
- **Control Systems**: PID controllers and feedback systems
- **Mathematical Foundations**: Linear algebra, transformations, and optimization

## Implementation Section

### Forward and Inverse Kinematics

```python
# kinematics.py
import numpy as np
import math
from typing import List, Tuple

class RobotKinematics:
    """Class for robot kinematics calculations"""

    def __init__(self, link_lengths: List[float]):
        """
        Initialize robot with link lengths

        Args:
            link_lengths: List of link lengths [L1, L2, ...]
        """
        self.link_lengths = link_lengths
        self.n_joints = len(link_lengths)

    def forward_kinematics_2d(self, joint_angles: List[float]) -> Tuple[float, float]:
        """
        Calculate end-effector position for 2D planar robot

        Args:
            joint_angles: List of joint angles in radians

        Returns:
            (x, y) end-effector position
        """
        if len(joint_angles) != self.n_joints:
            raise ValueError(f"Expected {self.n_joints} joint angles, got {len(joint_angles)}")

        x = 0.0
        y = 0.0
        cumulative_angle = 0.0

        for i, (angle, length) in enumerate(zip(joint_angles, self.link_lengths)):
            cumulative_angle += angle
            x += length * math.cos(cumulative_angle)
            y += length * math.sin(cumulative_angle)

        return x, y

    def inverse_kinematics_2d(self, target_x: float, target_y: float,
                            initial_angles: List[float] = None) -> List[float]:
        """
        Calculate joint angles for 2D planar robot to reach target position

        Args:
            target_x: Target x position
            target_y: Target y position
            initial_angles: Initial joint angles (for iterative methods)

        Returns:
            List of joint angles to reach target position
        """
        target_distance = math.sqrt(target_x**2 + target_y**2)

        # Check if target is reachable
        total_length = sum(self.link_lengths)
        if target_distance > total_length:
            # Target is out of reach, extend fully toward target
            angle_to_target = math.atan2(target_y, target_x)
            return [angle_to_target] + [0.0] * (self.n_joints - 1)

        # For a 2-link robot, we can solve analytically
        if self.n_joints == 2:
            L1, L2 = self.link_lengths

            # Use law of cosines to find second joint angle
            cos_theta2 = (L1**2 + L2**2 - target_distance**2) / (2 * L1 * L2)
            cos_theta2 = max(-1, min(1, cos_theta2))  # Clamp to valid range
            theta2 = math.acos(cos_theta2)

            # Find first joint angle
            k1 = L1 + L2 * math.cos(theta2)
            k2 = L2 * math.sin(theta2)

            theta1 = math.atan2(target_y, target_x) - math.atan2(k2, k1)

            return [theta1, theta2]

        # For more complex robots, use numerical methods (simplified Jacobian approach)
        else:
            # This is a simplified approach - in practice you'd use more sophisticated methods
            # like Jacobian inverse or pseudo-inverse
            if initial_angles is None:
                initial_angles = [0.0] * self.n_joints

            # Use gradient descent approach (simplified)
            angles = initial_angles.copy()
            learning_rate = 0.01
            max_iterations = 1000
            tolerance = 0.001

            for _ in range(max_iterations):
                current_x, current_y = self.forward_kinematics_2d(angles)
                error_x = target_x - current_x
                error_y = target_y - current_y
                error_magnitude = math.sqrt(error_x**2 + error_y**2)

                if error_magnitude < tolerance:
                    break

                # Simple Jacobian approximation (this is a very simplified version)
                # In practice, you'd calculate the actual Jacobian matrix
                for i in range(len(angles)):
                    # Perturb angle slightly to estimate gradient
                    angles[i] += 0.001
                    perturbed_x, perturbed_y = self.forward_kinematics_2d(angles)
                    dx = perturbed_x - current_x
                    dy = perturbed_y - current_y
                    angles[i] -= 0.001  # Restore

                    # Update angle based on gradient
                    gradient = dx * error_x + dy * error_y
                    angles[i] += learning_rate * gradient

            return angles

    def jacobian_2d(self, joint_angles: List[float]) -> np.ndarray:
        """
        Calculate the Jacobian matrix for a 2D planar robot

        Args:
            joint_angles: List of joint angles in radians

        Returns:
            2xN Jacobian matrix where N is number of joints
        """
        n = len(joint_angles)
        jacobian = np.zeros((2, n))

        # Calculate cumulative angles and positions
        cumulative_angle = 0.0
        x_cum = 0.0
        y_cum = 0.0

        for i, (angle, length) in enumerate(zip(joint_angles, self.link_lengths)):
            cumulative_angle += angle

            # Calculate position of current joint
            x_joint = x_cum + length * math.cos(cumulative_angle)
            y_joint = y_cum + length * math.sin(cumulative_angle)

            # Calculate partial derivatives (Jacobian elements)
            # dx/dtheta_i = -sum of lengths*cos(cumulative_angle) for joints i to n
            # dy/dtheta_i = sum of lengths*sin(cumulative_angle) for joints i to n
            for j in range(i, n):
                if j == i:
                    jacobian[0, i] -= length * math.sin(cumulative_angle)
                    jacobian[1, i] += length * math.cos(cumulative_angle)
                else:
                    # For subsequent joints, add their contribution
                    remaining_length = self.link_lengths[j]
                    remaining_angle = cumulative_angle + sum(joint_angles[i+1:j+1])
                    jacobian[0, i] -= remaining_length * math.sin(remaining_angle)
                    jacobian[1, i] += remaining_length * math.cos(remaining_angle)

        return jacobian

# Example usage
def kinematics_example():
    """Example of using kinematics functions"""
    # Create a 2-link robot with link lengths [1.0, 0.8]
    robot = RobotKinematics([1.0, 0.8])

    # Forward kinematics example
    joint_angles = [math.pi/4, math.pi/6]  # 45° and 30°
    end_x, end_y = robot.forward_kinematics_2d(joint_angles)
    print(f"Forward kinematics: Joint angles {joint_angles} -> End position ({end_x:.3f}, {end_y:.3f})")

    # Inverse kinematics example
    target_x, target_y = 1.2, 0.8
    solution = robot.inverse_kinematics_2d(target_x, target_y)
    print(f"Inverse kinematics: Target ({target_x}, {target_y}) -> Joint angles {[f'{angle:.3f}' for angle in solution]}")

    # Verify the solution
    check_x, check_y = robot.forward_kinematics_2d(solution)
    print(f"Verification: Forward kinematics gives ({check_x:.3f}, {check_y:.3f}), error = {math.sqrt((check_x-target_x)**2 + (check_y-target_y)**2):.6f}")

if __name__ == "__main__":
    kinematics_example()
```

### PID Controller Implementation

```python
# pid_controller.py
import time
from typing import Union, Callable
import numpy as np

class PIDController:
    """PID controller implementation for robotics applications"""

    def __init__(self, kp: float, ki: float, kd: float,
                 setpoint: float = 0.0,
                 output_limits: Tuple[float, float] = (-1.0, 1.0)):
        """
        Initialize PID controller

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            setpoint: Desired value
            output_limits: Tuple of (min_output, max_output)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits

        # Internal variables
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

        # Anti-windup parameters
        self._windup_guard = 20.0

    def update(self, process_variable: float, dt: float = None) -> float:
        """
        Update PID controller and return output

        Args:
            process_variable: Current measured value
            dt: Time step (if None, will calculate from time)

        Returns:
            Controller output
        """
        current_time = time.time()

        if dt is None:
            if self._last_time is None:
                dt = 0.01  # Default time step
            else:
                dt = current_time - self._last_time
        else:
            if dt <= 0:
                raise ValueError("dt must be positive")

        self._last_time = current_time

        # Calculate error
        error = self.setpoint - process_variable

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self._integral += error * dt
        # Anti-windup
        self._integral = np.clip(self._integral,
                                -self._windup_guard, self._windup_guard)
        i_term = self.ki * self._integral

        # Derivative term
        if dt > 0:
            derivative = (error - self._last_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative

        # Calculate output
        output = p_term + i_term + d_term

        # Apply output limits
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        # Store values for next iteration
        self._last_error = error

        return output

    def set_setpoint(self, setpoint: float):
        """Set new setpoint"""
        self.setpoint = setpoint
        # Reset integral when setpoint changes significantly
        if abs(setpoint - self.setpoint) > 0.1:
            self._integral = 0.0

    def set_tunings(self, kp: float, ki: float, kd: float):
        """Set new PID tuning parameters"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def reset(self):
        """Reset the PID controller"""
        self._last_error = 0.0
        self._integral = 0.0
        self._last_time = None

class RobotMotionController:
    """Robot motion controller using PID for position and velocity control"""

    def __init__(self):
        # PID controllers for x, y, and theta
        self.x_controller = PIDController(kp=2.0, ki=0.1, kd=0.05,
                                        output_limits=(-1.0, 1.0))
        self.y_controller = PIDController(kp=2.0, ki=0.1, kd=0.05,
                                        output_limits=(-1.0, 1.0))
        self.theta_controller = PIDController(kp=3.0, ki=0.1, kd=0.1,
                                            output_limits=(-1.0, 1.0))

    def control_to_pose(self, current_pose: Tuple[float, float, float],
                       target_pose: Tuple[float, float, float],
                       dt: float = 0.01) -> Tuple[float, float]:
        """
        Control robot to reach target pose

        Args:
            current_pose: (x, y, theta) of current robot pose
            target_pose: (x, y, theta) of target robot pose
            dt: Time step

        Returns:
            (linear_velocity, angular_velocity) commands
        """
        current_x, current_y, current_theta = current_pose
        target_x, target_y, target_theta = target_pose

        # Calculate position error in robot frame
        dx = target_x - current_x
        dy = target_y - current_y

        # Transform to robot frame
        error_x = dx * math.cos(-current_theta) - dy * math.sin(-current_theta)
        error_y = dx * math.sin(-current_theta) + dy * math.cos(-current_theta)

        # Calculate angular error (with wrap-around handling)
        angle_error = target_theta - current_theta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Update controllers
        v_x = self.x_controller.update(0, dt)  # Error is current error
        v_y = self.y_controller.update(0, dt)  # Error is current error
        omega = self.theta_controller.update(0, dt)  # Error is current error

        # Set setpoints for next iteration
        self.x_controller.set_setpoint(error_x)
        self.y_controller.set_setpoint(error_y)
        self.theta_controller.set_setpoint(angle_error)

        # Calculate linear and angular velocities
        linear_vel = math.sqrt(v_x**2 + v_y**2)
        angular_vel = omega

        return linear_vel, angular_vel

def pid_example():
    """Example of using PID controller for robot control"""
    import math

    # Create motion controller
    controller = RobotMotionController()

    # Simulate robot moving to target
    current_pose = (0.0, 0.0, 0.0)  # Start at origin, facing x-axis
    target_pose = (2.0, 1.0, math.pi/4)  # Target: (2,1) at 45 degrees

    print("PID Controller Example - Robot Motion Control")
    print(f"Starting pose: {current_pose}")
    print(f"Target pose: {target_pose}")
    print()

    # Simulate control loop
    for step in range(100):
        linear_vel, angular_vel = controller.control_to_pose(current_pose, target_pose, 0.01)

        # Simple kinematic model (unicycle)
        dt = 0.01
        x, y, theta = current_pose
        x += linear_vel * math.cos(theta) * dt
        y += linear_vel * math.sin(theta) * dt
        theta += angular_vel * dt

        current_pose = (x, y, theta)

        distance_to_target = math.sqrt((x - target_pose[0])**2 + (y - target_pose[1])**2)

        if step % 10 == 0:  # Print every 10 steps
            print(f"Step {step:2d}: Pose=({x:.3f}, {y:.3f}, {theta:.3f}), "
                  f"Vel=({linear_vel:.3f}, {angular_vel:.3f}), "
                  f"Distance to target: {distance_to_target:.3f}")

        if distance_to_target < 0.01:  # Close enough to target
            print(f"Target reached at step {step}")
            break

if __name__ == "__main__":
    pid_example()
```

### Trajectory Planning and Interpolation

```python
# trajectory_planning.py
import numpy as np
import math
from typing import List, Tuple
import matplotlib.pyplot as plt

class TrajectoryPlanner:
    """Class for generating robot trajectories"""

    def __init__(self):
        pass

    def linear_trajectory(self, start_pos: Tuple[float, float],
                         end_pos: Tuple[float, float],
                         duration: float, dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate linear trajectory between two points

        Args:
            start_pos: Starting position (x, y)
            end_pos: Ending position (x, y)
            duration: Total duration of trajectory
            dt: Time step

        Returns:
            (time_array, position_array, velocity_array)
        """
        t = np.arange(0, duration, dt)

        # Linear interpolation
        x_start, y_start = start_pos
        x_end, y_end = end_pos

        x = x_start + (x_end - x_start) * (t / duration)
        y = y_start + (y_end - y_start) * (t / duration)

        positions = np.column_stack((x, y))

        # Calculate velocities (numerical derivative)
        vx = np.gradient(x, dt)
        vy = np.gradient(y, dt)
        velocities = np.column_stack((vx, vy))

        return t, positions, velocities

    def cubic_trajectory(self, start_pos: float, start_vel: float,
                        end_pos: float, end_vel: float,
                        duration: float, dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate cubic polynomial trajectory (position, velocity, acceleration)

        Args:
            start_pos: Starting position
            start_vel: Starting velocity
            end_pos: Ending position
            end_vel: Ending velocity
            duration: Total duration
            dt: Time step

        Returns:
            (time_array, position_array, velocity_array)
        """
        t = np.arange(0, duration, dt)
        t_norm = t / duration  # Normalize time to [0, 1]

        # Cubic polynomial coefficients
        # q(t) = a0 + a1*t + a2*t^2 + a3*t^3
        a0 = start_pos
        a1 = start_vel * duration
        a2 = 3 * (end_pos - start_pos) - duration * (2 * start_vel + end_vel)
        a3 = 2 * (start_pos - end_pos) + duration * (start_vel + end_vel)

        # Calculate position
        q = a0 + a1 * t_norm + a2 * t_norm**2 + a3 * t_norm**3

        # Calculate velocity
        dq_dt_norm = a1 + 2 * a2 * t_norm + 3 * a3 * t_norm**2
        dq = dq_dt_norm / duration  # Convert back to real time

        return t, q, dq

    def quintic_trajectory(self, start_pos: float, start_vel: float, start_acc: float,
                          end_pos: float, end_vel: float, end_acc: float,
                          duration: float, dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate quintic polynomial trajectory (with acceleration constraints)

        Args:
            start_pos: Starting position
            start_vel: Starting velocity
            start_acc: Starting acceleration
            end_pos: Ending position
            end_vel: Ending velocity
            end_acc: Ending acceleration
            duration: Total duration
            dt: Time step

        Returns:
            (time_array, position_array, velocity_array)
        """
        t = np.arange(0, duration, dt)
        t_norm = t / duration  # Normalize time to [0, 1]

        # Quintic polynomial coefficients
        a0 = start_pos
        a1 = start_vel * duration
        a2 = 0.5 * start_acc * duration**2
        a3 = 10 * (end_pos - start_pos) - 6 * start_vel * duration - 3 * end_vel * duration - 0.5 * start_acc * duration**2 + 0.5 * end_acc * duration**2
        a4 = -15 * (end_pos - start_pos) + 8 * start_vel * duration + 7 * end_vel * duration + start_acc * duration**2 - 0.5 * end_acc * duration**2
        a5 = 6 * (end_pos - start_pos) - 3 * start_vel * duration - 3 * end_vel * duration - 0.5 * start_acc * duration**2 + 0.5 * end_acc * duration**2

        # Calculate position
        q = a0 + a1 * t_norm + a2 * t_norm**2 + a3 * t_norm**3 + a4 * t_norm**4 + a5 * t_norm**5

        # Calculate velocity
        dq_dt_norm = a1 + 2 * a2 * t_norm + 3 * a3 * t_norm**2 + 4 * a4 * t_norm**3 + 5 * a5 * t_norm**4
        dq = dq_dt_norm / duration  # Convert back to real time

        return t, q, dq

    def minimum_jerk_trajectory(self, start_pos: float, end_pos: float,
                               duration: float, dt: float = 0.01) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate minimum jerk trajectory (smooth motion)

        Args:
            start_pos: Starting position
            end_pos: Ending position
            duration: Total duration
            dt: Time step

        Returns:
            (time_array, position_array, velocity_array)
        """
        t = np.arange(0, duration, dt)
        t_norm = t / duration  # Normalize time to [0, 1]

        # Minimum jerk trajectory formula
        # q(t) = q0 + (qf - q0) * (10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5)
        q = start_pos + (end_pos - start_pos) * (10 * t_norm**3 - 15 * t_norm**4 + 6 * t_norm**5)

        # Calculate velocity
        dq_dt_norm = (end_pos - start_pos) * (30 * t_norm**2 - 60 * t_norm**3 + 30 * t_norm**4)
        dq = dq_dt_norm / duration  # Convert back to real time

        return t, q, dq

class PathPlanner:
    """Class for path planning through waypoints"""

    def __init__(self):
        pass

    def bezier_curve(self, control_points: List[Tuple[float, float]],
                    num_points: int = 100) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate Bezier curve through control points

        Args:
            control_points: List of (x, y) control points
            num_points: Number of points to generate

        Returns:
            (x_array, y_array) of curve points
        """
        control_points = np.array(control_points)
        n = len(control_points) - 1  # Degree of curve

        t = np.linspace(0, 1, num_points)

        # Calculate binomial coefficients
        def binomial_coefficient(n, k):
            if k < 0 or k > n:
                return 0
            if k == 0 or k == n:
                return 1
            return math.factorial(n) // (math.factorial(k) * math.factorial(n - k))

        # Calculate Bezier curve points
        x = np.zeros(num_points)
        y = np.zeros(num_points)

        for i in range(n + 1):
            coeff = binomial_coefficient(n, i)
            blend = coeff * (t**i) * ((1 - t)**(n - i))
            x += blend * control_points[i, 0]
            y += blend * control_points[i, 1]

        return x, y

    def path_following_controller(self, waypoints: List[Tuple[float, float]],
                                 current_pos: Tuple[float, float],
                                 look_ahead_distance: float = 1.0) -> Tuple[float, float]:
        """
        Pure pursuit path following controller

        Args:
            waypoints: List of (x, y) waypoints
            current_pos: Current (x, y) position
            look_ahead_distance: Look-ahead distance for target point selection

        Returns:
            (target_x, target_y) closest point on path
        """
        if len(waypoints) < 2:
            return current_pos

        current_x, current_y = current_pos
        waypoints = np.array(waypoints)

        # Find the closest point on the path
        min_distance = float('inf')
        closest_idx = 0

        for i in range(len(waypoints) - 1):
            # Calculate distance to line segment
            p1 = waypoints[i]
            p2 = waypoints[i + 1]

            # Vector from p1 to p2
            line_vec = p2 - p1
            line_len = np.linalg.norm(line_vec)
            if line_len == 0:
                continue

            # Normalize line vector
            line_vec_norm = line_vec / line_len

            # Vector from p1 to current position
            vec_to_current = np.array([current_x, current_y]) - p1

            # Project vec_to_current onto line
            projection_length = np.dot(vec_to_current, line_vec_norm)
            projection_length = np.clip(projection_length, 0, line_len)

            # Find closest point on line segment
            closest_point = p1 + projection_length * line_vec_norm

            # Calculate distance
            distance = np.linalg.norm(np.array([current_x, current_y]) - closest_point)

            if distance < min_distance:
                min_distance = distance
                closest_idx = i

        # Select target point ahead on the path
        p1 = waypoints[closest_idx]
        p2 = waypoints[closest_idx + 1]

        # Direction vector of the path segment
        path_dir = p2 - p1
        path_len = np.linalg.norm(path_dir)

        if path_len == 0:
            return tuple(p1)

        path_dir_norm = path_dir / path_len

        # Calculate how far along the segment to place the look-ahead point
        vec_to_p1 = p1 - np.array([current_x, current_y])
        projection_on_segment = np.dot(vec_to_p1, path_dir_norm)

        # Target point is look_ahead_distance ahead of closest point
        look_ahead_pos = p1 + path_dir_norm * min(path_len, projection_on_segment + look_ahead_distance)

        return float(look_ahead_pos[0]), float(look_ahead_pos[1])

def trajectory_example():
    """Example of trajectory planning"""
    planner = TrajectoryPlanner()

    print("Trajectory Planning Examples")
    print("=" * 40)

    # Example 1: Linear trajectory
    print("\n1. Linear Trajectory:")
    t, pos, vel = planner.linear_trajectory((0, 0), (3, 4), 5.0)
    print(f"   From (0,0) to (3,4) in 5 seconds")
    print(f"   Final position: ({pos[-1, 0]:.3f}, {pos[-1, 1]:.3f})")
    print(f"   Final velocity: ({vel[-1, 0]:.3f}, {vel[-1, 1]:.3f})")

    # Example 2: Cubic trajectory
    print("\n2. Cubic Trajectory:")
    t, pos, vel = planner.cubic_trajectory(0, 0, 5, 0, 3.0)  # Start at 0, end at 5, both velocities 0
    print(f"   Position from 0 to 5 in 3 seconds")
    print(f"   Start pos/vel: {pos[0]:.3f}/{vel[0]:.3f}")
    print(f"   End pos/vel: {pos[-1]:.3f}/{vel[-1]:.3f}")

    # Example 3: Minimum jerk trajectory
    print("\n3. Minimum Jerk Trajectory:")
    t, pos, vel = planner.minimum_jerk_trajectory(0, 10, 4.0)
    print(f"   Position from 0 to 10 in 4 seconds")
    print(f"   Start pos/vel: {pos[0]:.3f}/{vel[0]:.3f}")
    print(f"   End pos/vel: {pos[-1]:.3f}/{vel[-1]:.3f}")

def path_planning_example():
    """Example of path planning"""
    path_planner = PathPlanner()

    print("\nPath Planning Examples")
    print("=" * 40)

    # Example: Bezier curve
    print("\n1. Bezier Curve:")
    control_points = [(0, 0), (1, 2), (3, 3), (4, 1)]
    x, y = path_planner.bezier_curve(control_points, num_points=50)
    print(f"   Generated Bezier curve through points: {control_points}")
    print(f"   Curve has {len(x)} points")

    # Example: Path following
    print("\n2. Path Following (Pure Pursuit):")
    waypoints = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 0)]
    current_pos = (0.5, 0.2)
    target = path_planner.path_following_controller(waypoints, current_pos)
    print(f"   Waypoints: {waypoints}")
    print(f"   Current pos: {current_pos}")
    print(f"   Look-ahead target: {target}")

if __name__ == "__main__":
    trajectory_example()
    path_planning_example()
```

### Sensor Fusion and State Estimation

```python
# sensor_fusion.py
import numpy as np
import math
from typing import Tuple, List
import matplotlib.pyplot as plt

class KalmanFilter:
    """Simple Kalman Filter implementation for state estimation"""

    def __init__(self, state_dim: int, measurement_dim: int):
        """
        Initialize Kalman Filter

        Args:
            state_dim: Dimension of state vector
            measurement_dim: Dimension of measurement vector
        """
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        # State vector (will be initialized later)
        self.x = np.zeros((state_dim, 1))

        # Error covariance matrix
        self.P = np.eye(state_dim) * 1000  # Large initial uncertainty

        # Process noise covariance
        self.Q = np.eye(state_dim) * 0.1

        # Measurement noise covariance
        self.R = np.eye(measurement_dim) * 1.0

        # State transition model (will be set by user)
        self.F = np.eye(state_dim)

        # Measurement model (will be set by user)
        self.H = np.zeros((measurement_dim, state_dim))

    def predict(self):
        """Prediction step of Kalman Filter"""
        # Predict state: x = F * x
        self.x = self.F @ self.x

        # Predict error covariance: P = F * P * F^T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z: np.ndarray):
        """
        Update step of Kalman Filter

        Args:
            z: Measurement vector
        """
        # Calculate innovation: y = z - H * x
        y = z.reshape(-1, 1) - self.H @ self.x

        # Calculate innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Calculate Kalman gain: K = P * H^T * S^(-1)
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state: x = x + K * y
        self.x = self.x + K @ y

        # Update error covariance: P = (I - K * H) * P
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

class RobotStateEstimator:
    """Robot state estimator using sensor fusion"""

    def __init__(self):
        # State: [x, y, theta, vx, vy, omega]
        # x, y: position
        # theta: orientation
        # vx, vy: linear velocities
        # omega: angular velocity
        self.kf = KalmanFilter(state_dim=6, measurement_dim=3)  # x, y, theta measurements

        # Initialize state transition model (constant velocity model)
        dt = 0.01  # Time step
        self.kf.F = np.array([
            [1, 0, 0, dt, 0,  0],    # x = x + vx*dt
            [0, 1, 0, 0,  dt, 0],    # y = y + vy*dt
            [0, 0, 1, 0,  0,  dt],   # theta = theta + omega*dt
            [0, 0, 0, 1,  0,  0],    # vx = vx (constant)
            [0, 0, 0, 0,  1,  0],    # vy = vy (constant)
            [0, 0, 0, 0,  0,  1]     # omega = omega (constant)
        ])

        # Initialize measurement model (measure x, y, theta)
        self.kf.H = np.array([
            [1, 0, 0, 0, 0, 0],  # Measure x
            [0, 1, 0, 0, 0, 0],  # Measure y
            [0, 0, 1, 0, 0, 0]   # Measure theta
        ])

        # Initialize with some values
        self.kf.x = np.array([[0], [0], [0], [0], [0], [0]])  # Start at origin

    def update_with_measurement(self, x_meas: float, y_meas: float, theta_meas: float) -> Tuple[float, float, float]:
        """
        Update state estimate with new measurement

        Args:
            x_meas: Measured x position
            y_meas: Measured y position
            theta_meas: Measured orientation

        Returns:
            (x, y, theta) estimated state
        """
        # Create measurement vector
        z = np.array([x_meas, y_meas, theta_meas])

        # Perform prediction and update steps
        self.kf.predict()
        self.kf.update(z)

        # Return estimated state
        return float(self.kf.x[0, 0]), float(self.kf.x[1, 0]), float(self.kf.x[2, 0])

    def predict_motion(self, linear_vel: float, angular_vel: float, dt: float = 0.01):
        """
        Predict robot motion based on control inputs

        Args:
            linear_vel: Linear velocity command
            angular_vel: Angular velocity command
            dt: Time step
        """
        # Update state transition matrix with current velocities
        self.kf.F[0, 3] = dt  # dx += vx*dt
        self.kf.F[1, 4] = dt  # dy += vy*dt
        self.kf.F[2, 5] = dt  # dtheta += omega*dt

        # Update velocity estimates based on control inputs
        # This is a simplified model - in reality, you'd have a more complex motion model
        current_theta = self.kf.x[2, 0]
        self.kf.x[3, 0] = linear_vel * math.cos(current_theta)  # vx
        self.kf.x[4, 0] = linear_vel * math.sin(current_theta)  # vy
        self.kf.x[5, 0] = angular_vel  # omega

def sensor_fusion_example():
    """Example of sensor fusion using Kalman Filter"""
    estimator = RobotStateEstimator()

    print("Sensor Fusion Example - Kalman Filter")
    print("=" * 50)

    # Simulate robot movement with noisy measurements
    true_positions = []
    measured_positions = []
    estimated_positions = []

    # Initial conditions
    true_x, true_y, true_theta = 0.0, 0.0, 0.0
    linear_vel = 0.5  # m/s
    angular_vel = 0.1  # rad/s

    dt = 0.01  # 100 Hz

    for step in range(500):  # Simulate 5 seconds
        # Update true position (simulated robot motion)
        true_x += linear_vel * math.cos(true_theta) * dt
        true_y += linear_vel * math.sin(true_theta) * dt
        true_theta += angular_vel * dt

        # Simulate noisy measurements
        noise_std = 0.1
        meas_x = true_x + np.random.normal(0, noise_std)
        meas_y = true_y + np.random.normal(0, noise_std)
        meas_theta = true_theta + np.random.normal(0, noise_std * 0.1)

        # Update estimator with measurement
        est_x, est_y, est_theta = estimator.update_with_measurement(meas_x, meas_y, meas_theta)

        # Store for plotting
        true_positions.append((true_x, true_y))
        measured_positions.append((meas_x, meas_y))
        estimated_positions.append((est_x, est_y))

        # Print progress
        if step % 100 == 0:
            print(f"Step {step}: True=({true_x:.3f}, {true_y:.3f}), "
                  f"Meas=({meas_x:.3f}, {meas_y:.3f}), "
                  f"Est=({est_x:.3f}, {est_y:.3f})")

    print(f"\nFinal results after {len(true_positions)*dt:.1f} seconds:")
    final_true = true_positions[-1]
    final_meas = measured_positions[-1]
    final_est = estimated_positions[-1]
    print(f"True position: ({final_true[0]:.3f}, {final_true[1]:.3f})")
    print(f"Measured position: ({final_meas[0]:.3f}, {final_meas[1]:.3f})")
    print(f"Estimated position: ({final_est[0]:.3f}, {final_est[1]:.3f})")

if __name__ == "__main__":
    sensor_fusion_example()
```

## Additional Resources

- [Python Robotics Library](https://pythonrobotics.github.io/)
- [NumPy for Robotics](https://numpy.org/)
- [SciPy for Scientific Computing](https://scipy.org/)
- [Robotics Algorithms in Python](https://github.com/AtsushiSakai/PythonRobotics)

## Quiz Questions

1. What are the main components of a PID controller and their functions?
2. How does forward kinematics differ from inverse kinematics?
3. What is the purpose of sensor fusion in robotics?