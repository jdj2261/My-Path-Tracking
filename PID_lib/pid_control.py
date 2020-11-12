import time

class PID_Control:
    def __init__(self, P=0.2, I=0.0, D=0.0, dt=0.1, current_time=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.dt = dt
        self.current_time = current_time
        self.last_time = self.current_time
        self.clear()

    def clear(self):

        self.P_term = 0.0
        self.I_term = 0.0
        self.D_term = 0.0
        self.last_error = 0
        self.output = 0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

    def update(self, target, current, current_time):
        error = target - current

        self.current_time = current_time
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        # print(delta_time)
        # print("last time : {}, current_time : {} ".format(self.last_time, self.current_time))

        if (delta_time >= self.dt):
            # print("yes")

            self.P_term = self.Kp * error
            self.I_term += self.Ki * error * delta_time 

            if (self.I_term < -self.windup_guard):
                self.I_term = -self.windup_guard
            elif (self.I_term > self.windup_guard):
                self.I_term = self.windup_guard

            self.D_term = 0.0
            if delta_time > 0:
                self.D_term = self.Kd * (delta_error / delta_time)

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
          
            self.last_error = error
            self.output = self.P_term + self.I_term + self.D_term
            # time.sleep(0.01)
            
            return float(self.output) 

    def setWindup(self, windup):
        self.windup_guard = windup