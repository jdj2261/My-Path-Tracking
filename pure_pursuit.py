"""
Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import numpy as np
import math
import matplotlib.pyplot as plt

# Parameters
k = 0.1  # look forward gain
Lfc = 2.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle

show_animation = True

class State:
    """
    current robot pose : x, y, yaw, 
    current robot velocity : v

    robot's rear pose : rear_x, rear_y
    """
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
    """
    Current States Add List
    """

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


def proportional_control(target, current):
    """
    PID control
    """
    a = Kp * (target - current)
    return a

class TargetCourse:
    """
    Select Target Index

    cx : course x
    cy : course y
    """
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index

            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]

            distance = np.hypot(dx, dy)
            ind = np.argmin(distance)
            # print(distance, ind)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            print(ind)
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            # print("rear_x : {}, rear_y : {}\t".format(state.x, state.y))
            
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
 

                if distance_this_index < distance_next_index:
                    # print("Current SMALL ")
                    break
                # print("next_distance : {}, current_distance : {}".format(
                # distance_next_index, distance_this_index))
                # print("PLUS")
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index

            self.old_nearest_point_index = ind
          
        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            # print("cx : {}, cy : {}".format(self.cx[ind], self.cy[ind]))

            ind += 1


        # print(state.calc_distance(self.cx[ind], self.cy[ind]))
        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    # print(pind, ind)
    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    # print(state.yaw)
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
    # print("YAW : {}, ALPHA : {}".format(state.yaw, alpha))

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def plot_arrow(x, y, yaw, length=0.5, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def deg2rad( degree ) :
	return degree * math.pi / 180.0

def rad2deg (radian):
    return radian * 180.0 / math.pi

def main():
    #  target course
    cx = np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 10.0 for ix in cx]

    target_speed = 5.0 / 3.6  # [m/s]

    T = 200.0  # max simulation time

    # initial state
    state = State(x=10.0, y=10.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)


    target_ind, _ = target_course.search_target_index(state)

    while T >= time and lastIndex > target_ind:
        # print(target_course.old_nearest_point_index)
        # Calc control input
        ai = proportional_control(target_speed, state.v)

        di, target_ind = pure_pursuit_steer_control(
            state, target_course, target_ind)

        state.update(ai, di)  # Control vehicle
        # print(state.v * 3.6, ai,rad2deg(di))
        time += dt
        states.append(time, state)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)

            plt.plot(cx, cy, "ro", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            # print(state.x, state.y)
            # print(target_ind)
            plt.plot(cx[target_ind], cy[target_ind], "go", label="target")
            plt.legend()
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)
        

    # Test
    assert lastIndex >= target_ind, "Cannot goal"

    print("Show Graph")
    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
