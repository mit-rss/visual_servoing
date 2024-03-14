class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute_control_signal(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        kp_file_path = 'kp.txt'
        with open(kp_file_path, 'a') as file:
            file.write(f'{self.kp*error} ')

        ki_file_path = 'ki.txt'
        with open(ki_file_path, 'a') as file:
            file.write(f'{self.ki*error} ')

        kd_file_path = 'kd.txt'
        with open(kd_file_path, 'a') as file:
            file.write(f'{self.kd*error} ')

        error_file_path = 'error.txt'
        with open(error_file_path, 'a') as file:
            file.write(f'{error} ')

        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error

        return control_signal
