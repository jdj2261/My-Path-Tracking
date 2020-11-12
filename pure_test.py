import numpy as np
import math
import time
import matplotlib.pyplot as plt
from PID_lib.pid_control import PID_Control

# Parameters
Lf_k = 0.1  # look forward gain
Ld = 5.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.01  # [s] time tick
Wheel_Base = 2.9  # [m] wheel base of vehicle
target_speed = 5.0 / 3.6 #[m/s]

Max_Time = 100.0

show_animation = True

class PlotLib:
    """
    pyplot 
    """    
    def __init__(self):
        plt.cla()

    def draw(self, x, y, color="r", label="Test", xlabel="x[m]", ylabel="y[m]"):
        plt.plot(x, y, color, label=label)
        plt.legend()
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)

    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                    fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)

    @staticmethod
    def show():   
        plt.axis("equal")
        plt.grid(True) 
        plt.show()

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v 
        self.rear_x = self.x - ((Wheel_Base / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((Wheel_Base / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) #* dt 
        self.y += self.v * math.sin(self.yaw) #* dt
        self.yaw += self.v / Wheel_Base * math.tan(delta) #* dt
        self.v += a * dt
        self.rear_x = self.x - ((Wheel_Base / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((Wheel_Base / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None
    
    def search_target_index(self, state):

        # 처음 target index는 현재 포인트와 가장 가까운 index
        if self.old_nearest_point_index is None:
            dx = [state.rear_x - index_cx for index_cx in self.cx]
            dy = [state.rear_y - index_cy for index_cy in self.cy]

            distance = np.hypot(dx, dy)
            index = np.argmin(distance)
            self.old_nearest_point_index = index
        # 이후에 
        else:
            index = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[index], self.cy[index])

            while True:
                distance_next_index = state.calc_distance(self.cx[index + 1], self.cy[index + 1])

                if distance_this_index < distance_next_index:
                    break
                index = index + 1 if (index + 1) < len(self.cx) else index
                distance_this_index = distance_next_index
            self.old_nearest_point_index = index

        Lf_d = Lf_k * state.v + Ld

        while Lf_d > state.calc_distance(self.cx[index], self.cy[index]):
            if (index + 1) >= len(self.cx):
                break
            index += 1
        
        return Lf_d, index

class PurePursuit:
    def __init__(self):
        pass
    
    @staticmethod
    def pure_pursuit_steer_control(state, trajectory):
        Ld, target_index = trajectory.search_target_index(state)

        if target_index < len(trajectory.cx):
            target_x = trajectory.cx[target_index]
            target_y = trajectory.cy[target_index]
        else:
            target_x = trajectory.cx[-1]
            target_y = trajectory.cy[-1]
            target_index = len(trajectory.cx) - 1
        
        alpha = math.atan2(target_y - state.rear_y, target_x - state.rear_x) - state.yaw
        delta = math.atan2(2.0 * Wheel_Base * math.sin(alpha) / Ld , 1.0)

        return delta, target_index

    @staticmethod
    def rad2deg (radian):
        return radian * 180.0 / math.pi

    @staticmethod
    def deg2rad(degree) :
	    return degree * math.pi / 180.0

def main():
    cx = np.arange(0,50,0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    # cy2 = [math.cos(ix / 5.0) * ix / 10.0 for ix in cx]

    state = State(x=3.0, y=3.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    duration_time = 0.0

    states = States()
    states.append(duration_time, state)

    course = TargetCourse(cx, cy)

    _, target_index = course.search_target_index(state)

    pid_control = PID_Control(P=10.0, I=0.0, D=0.0, dt=dt)
    plt_lib = PlotLib()


    while Max_Time >= duration_time and lastIndex > target_index:
        current_time = time.time()

        pid_output = pid_control.update(target_speed, state.v, current_time)
        # print(pid_output)
        delta, target_index = PurePursuit.pure_pursuit_steer_control(state, course)

        state.update(pid_output, delta)

        print(target_index)
        duration_time += dt
        states.append(duration_time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            # plot_arrow(state.x, state.y, state.yaw)

            plt.plot(cx, cy, "ro", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            # print(state.x, state.y)
            # print(target_ind)
            plt.plot(cx[target_index], cy[target_index], "go", label="target")
            plt.legend()
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(dt)

        # plt_lib.draw(cx,cy, color = "ro", label="course")
        # plt_lib.draw(states.x, states.y, color = "-b", label="trajectory")
        # plt_lib.draw(cx[target_index], cy[target_index], color = "go", label="target")
        # plt.pause(0.001)
        # time.sleep(dt)

if __name__ == "__main__":
    main()
