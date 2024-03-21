class PIDController_Positional:
    def __init__(self, kp, ki, kd, set_point, max, min, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point
        self.max = max
        self.min = min
        self.dt = dt
        self.error_sum = 0
        self.last_error = 0

    # Measurement represents the current state
    def update(self, measurement):
        # calculation error
        error = self.set_point - measurement
        # scale error
        p_term = self.kp * error
        # integration error
        self.error_sum += error * self.dt
        i_term = self.ki * self.error_sum
        # Differential error
        delta_error = (error - self.last_error)/self.dt
        d_term = self.kd * delta_error
        # Obtain the target value of the output and set the range
        output = p_term + i_term + d_term
        if (output > self.max):
            output = self.max
        elif (output < self.min):
            output = self.min
        self.last_error = error
        return output

    def reset_setpoint(self,new_setpoint):
        self.set_point = new_setpoint


class PIDController_Incremental:
    def __init__(self, kp, ki, kd, set_point, max, min, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point
        self.max = max
        self.min = min
        self.dt = dt
        self.last_error = 0
        self.last_output = 0

    def update(self, measurement):
        error = self.set_point - measurement
        p_increment = self.kp * (error - self.last_error)
        i_increment = self.ki * error * self.dt
        d_increment = self.kd * (error - self.last_error) / self.dt
        output_increment = p_increment + i_increment + d_increment
        output = self.last_output + output_increment
        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min
        self.last_error = error
        self.last_output = output

        return output

    def reset_setpoint(self,new_setpoint):
        self.set_point = new_setpoint