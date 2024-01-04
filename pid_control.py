class PIDControl:
    def __init__(self, Kp=0.25, Ki=0.0, Kd=0.1, integral_upper_limit=20, integral_lower_limit=-20):  
        self.Kp = Kp  
        self.Ki = Ki  
        self.Kd = Kd  
        self.integral_upper_limit = integral_upper_limit  
        self.integral_lower_limit = integral_lower_limit  
        self.prev_error = 0  
        self.integral = 0  
  
    def update(self, setpoint, actual_position, dt):  
        error = setpoint - actual_position  
        self.integral += error * dt  
        derivative = self.Kd * (error - self.prev_error) / dt  
        self.prev_error = error  
        proportional = self.Kp * error 
        integral = self.Ki * self.clip(self.integral, self.integral_lower_limit, self.integral_upper_limit)
        control_increment = proportional + derivative + integral
        print("[PID]: 比例:{:.3f}, 积分:{:.3f}, 微分:{:.3f}, error:{:.3f}, p: {:.3f}"\
            .format(proportional,integral, derivative,error,control_increment))
        return control_increment  
  
    @staticmethod  
    def clip(value, lower_limit, upper_limit):  
        if value < lower_limit:  
            # print(f"lower_limit:{lower_limit}, upper_limit:{upper_limit}")
            return lower_limit  
        elif value > upper_limit:  
            # print(f"lower_limit:{lower_limit}, upper_limit:{upper_limit}")
            return upper_limit  
        else:  
            return value
    
    def clear(self):  
        self.integral = 0  
        self.prev_error = 0