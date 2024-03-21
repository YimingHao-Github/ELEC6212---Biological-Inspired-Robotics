from config import RobotConfig
from Model import DifferentialDriveRobot
from PIDCtrl import PIDController_Positional,PIDController_Incremental
from TrajectoryPoint import TrajectoryPoint
import math
import matplotlib.pyplot as plt

def calculate_distance_and_angle(current_x, current_y, target_x, target_y):
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.sqrt(dx**2 + dy**2)
    angle_to_target = math.atan2(dy, dx)
    return distance, angle_to_target

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

# Create a configuration instance and initialize parameters
robot_config = RobotConfig()

x_history = []
y_history = []

# TrajectoryPoint
# Randomly defined target points, just for demonstration purposes
points = []
points.append(TrajectoryPoint(13,15))
points.append(TrajectoryPoint(17,23))
points.append(TrajectoryPoint(28,39))
points.append(TrajectoryPoint(37,19))
points.append(TrajectoryPoint(17,49))

# Create robot instance
robot = DifferentialDriveRobot(robot_config.v0_left, robot_config.v0_right, robot_config.wheel_track,
                               robot_config.x0, robot_config.y0, robot_config.theta0, robot_config.Ts, robot_config.wheel_radius)

# Define target distance and angle
target_distance_velocity = 0.0
target_angular_velocity = 0.0
# Define vehicle wheel speed
target_left_velocity = 0.0
target_right_velocity = 0.0

# Create PID controller instances for distance and angular velocity, and wheel speed PID controller instances
# It's just a randomly defined parameter, not set according to the actual modeling, anyway it can reach the target point :)
linear_pid = PIDController_Positional(kp=0.05, ki=0.1, kd=0.05, set_point=target_distance_velocity, max = 0.01, min =-0.01, dt = robot_config.Ts)
angular_pid = PIDController_Positional(kp=0.01, ki=0, kd=0.0, set_point=target_angular_velocity, max = 500, min = -500, dt = robot_config.Ts)
left_velocity_pid = PIDController_Incremental(kp=0.01, ki=0, kd=0.0, set_point=target_left_velocity, max = 0.1, min =-0.1, dt = robot_config.Ts)
right_velocity_pid = PIDController_Incremental(kp=0.01, ki=0, kd=0.0, set_point=target_right_velocity, max = 0.1, min = -0.1, dt = robot_config.Ts)

for point in points:
    while(True):
        distance, angle_to_target = calculate_distance_and_angle(robot.x, robot.y, point.x, point.y)
        target_theta = normalize_angle(angle_to_target)
        angle_error = normalize_angle(target_theta - robot.theta)
        # Use PID controller to obtain new distance and angle commands
        distance_command = linear_pid.update(distance)
        angular_command = angular_pid.update(angle_error)
        # Distance variable velocity
        # (you can write a distance velocity mapping function of another type, but I haven't written it here.:( I will update it later when I have time)
        v_command = distance_command/robot.Ts
        omega_command = angular_command/robot.Ts
        # Update left and right wheel speed commands based on linear and angular speed commands
        target_left_velocity = v_command - omega_command * robot.wheel_track / 2.0
        target_right_velocity = v_command + omega_command * robot.wheel_track / 2.0
        left_velocity_pid.reset_setpoint(target_left_velocity)
        left_velocity_command = left_velocity_pid.update(robot.v_left)
        right_velocity_pid.reset_setpoint(target_right_velocity)
        right_velocity_command = right_velocity_pid.update(robot.v_right)
        # Update the left and right wheel speeds based on the left and right wheel speed command
        robot.v_left = robot.v_left + left_velocity_command
        robot.v_right = robot.v_right + right_velocity_command
        print("left:%f right:%f" % (robot.v_left, robot.v_right))
        # Update the status of the robot
        robot.update_state()
        # Recalculate distance and angle errors
        distance, tempppppp = calculate_distance_and_angle(robot.x, robot.y, point.x, point.y)
        if distance < 0.1:  # If the distance is less than a certain threshold, it is considered that the target point has been reached
            print("Reached point %f，%f" % (robot.x, robot.y))
            break
        # The following section records parameters for drawing
        x_history.append(robot.x)
        y_history.append(robot.y)

# Draw trajectories using Matplotlib
plt.figure(figsize=(8, 6))
plt.plot(x_history, y_history, label='Robot Path', linewidth=2)
plt.scatter(x_history, y_history, c='red', s=10)
plt.xlabel('x position ')
plt.ylabel('y position')
plt.title('Robot Trajectory')
plt.axis('equal')
plt.legend()
plt.grid(True)
plt.show()


