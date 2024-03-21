import math
import random
# v_left：左轮速度
# v_right：右轮速度
# wheel_track：轮距
# x：x轴位置
# y：y轴位置
# theta：朝向角度
# v：线速度
# omega：角速度
# Ts：采样时间
# wheel_radius:轮子半径

class DifferentialDriveRobot:
    def __init__(self, v_left, v_right, wheel_track, x0, y0, theta0, Ts, wheel_radius):
        self.v_left = v_left
        self.v_right = v_right
        self.wheel_track = wheel_track
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.Ts = Ts
        self.omega_left_wheel = v_left/wheel_radius
        self.omega__right_wheel = v_right/wheel_radius
        self.wheel_radius = wheel_radius
        self.update_state()

    def update_state(self):
        # Calculate linear velocity and angular velocity
        self.v = (self.v_right + self.v_left) / 2.0
        self.omega = (self.v_right - self.v_left) / self.wheel_track

        # Update position and orientation
        delta_x = self.v * math.cos(self.theta) * self.Ts
        delta_y = self.v * math.sin(self.theta) * self.Ts
        delta_theta = self.omega * self.Ts

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        # Keep theta within the range of - pi to pi
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        # Update angular velocity based on velocity
        self.omega_left_wheel = self.v_left/self.wheel_radius
        self.omega__right_wheel = self.v_right/self.wheel_radius

