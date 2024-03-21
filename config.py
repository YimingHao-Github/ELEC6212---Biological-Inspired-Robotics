import random

class RobotConfig:
    def __init__(self):
        self.wheel_track = 0.5 # wheel track
        self.v0_left = 0.0  # Left wheel initial speed
        self.v0_right = 0.0  # Right wheel initial speed
        self.x0 = 0.0  # Initial position x-axis coordinates
        self.y0 = 0.0  # Initial position y-axis coordinates
        self.theta0 = 0.0  # Initial orientation
        self.Ts = 0.01  # control frequency
        self.car_weight = 5.0  # weight
        self.wheel_radius = 1 # wheel radius

    def friction_coefficient(self):
        # 摩擦系数生成函数
        return random.uniform(0.4, 0.6)
