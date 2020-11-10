import numpy as np
import math
import time
import matplotlib.pyplot as plt
from PID_lib import pid_control

# Parameters
k = 0.1  # look forward gain
Lfc = 5.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle


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
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt 
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

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

class TargetIndex:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None
    
    def search_target_index(self, state):

        if self.old_nearest_point_index is None:
            dx = [state.rear_x - index_cx for index_cx in self.cx]
            dy = [state.rear_y - index_cy for index_cy in self.cy]

            distance = np.hypot(dx, dy)
            index = np.argmin(distance)
            self.old_nearest_point_index = index
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

class PurePursuit:
    def __init__(self):
        pass
    
    @staticmethod
    def pure_pursuit_steer_control(state, trajectory, pind):
        pass

    @staticmethod
    def rad2deg (radian):
        return radian * 180.0 / math.pi

    @staticmethod
    def deg2rad(degree) :
	    return degree * math.pi / 180.0

def test():
    X = np.linspace(-np.pi, np.pi, 256, endpoint=True)
    C,S = np.cos(X), np.sin(X)

    plt.plot(X,C)
    plt.plot(X,S)

    plt.show()

def main():
    cx = np.arange(0,100,0.5)
    cy = [math.sin(ix / 5.0) * ix / 10.0 for ix in cx]
    # cy2 = [math.cos(ix / 5.0) * ix / 10.0 for ix in cx]
    Max_Time = 1000
    dt_time = 0.0
    # plt_lib = PlotLib()
    while Max_Time >= dt_time :
        dt_time += dt
        time.sleep(dt)

        print(dt_time)


if __name__ == "__main__":
    main()
