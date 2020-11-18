#!/usr/bin/env python
# -*- coding: utf-8 -*-


'''
Created Date: Nov 10. 2020
Copyright: UNMANNED SOLUTION
Author: Dae Jong Jin 
Description: Pure Pursuit Algorithm, PID Control Test 
'''

import numpy as np
import time
import matplotlib.pyplot as plt

# Libraries
from PLT_lib.plotlib import PlotLib
from PID_lib.pid_control import PIDControl
from PurePursuit_lib.pure_pursuit import State, States, TargetCourse, PurePursuit

# Parameters
#############################################
# Pure Pursuit Param #
#############################################
Wheel_Base = 2.9  # [m] wheel base of vehicle
Lf_k = 1.0  # look forward gain
Ld = 3.0  # [m] look-ahead distance

#############################################
# PID Param #
#############################################
Kp = 1.0  # speed proportional gain
Kd = 0.0
Ki = 0.0
target_speed = 1.0 / 3.6 #[m/s]
#############################################

dt = 0.01  # [s] time tick
show_animation = True

def main():
    cx = np.arange(0,100,0.5)
    cy = [np.sin(ix / 5) * ix / 2.0 for ix in cx]

    state = State(Wheel_Base, x=3.0, y=3.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    duration_time = 0.0

    current_time = time.time()
    pre_time = current_time

    states = States()
    states.append(duration_time, state)

    course = TargetCourse(cx, cy, Ld, Lf_k)
    _, target_index = course.search_target_index(state)

    pid_control = PIDControl(P=Kp, I=Ki, D=Kd, dt=dt)

    while target_index < lastIndex:
        current_time = time.time()
        delta_time = current_time - pre_time

        if pre_time != current_time:

            pid_output = pid_control.update(target_speed, state.v, delta_time)
            # print(pid_output)
            delta, target_index = PurePursuit.pure_pursuit_steer_control(state, course)
            state.update(pid_output, delta, delta_time)

            print("Target Index : {}\t".format(target_index), end=" ")
            print("Current Vel : {0:.4f}, Current Yaw : {1:.2f}".format(state.v*3.6, state.yaw))  


            duration_time += delta_time
            states.append(duration_time, state)

            pre_time = current_time

            if show_animation:  # pragma: no cover
                plt_lib = PlotLib(title = "Speed[km/h]:" + str(state.v * 3.6)[:6])
                plt_lib.draw(cx,cy, color = "ro", label="course")
                plt_lib.draw(states.x, states.y, color = "-b", label="trajectory")
                plt_lib.draw(cx[target_index], cy[target_index], color = "go", label="target")
                plt_lib.plot_arrow(state.x, state.y, state.yaw)
                plt_lib.set_legend()
                plt_lib.sleep(dt)

            else:
                time.sleep(dt)
        # Test
    assert lastIndex >= target_index, "Cannot goal"

    print("Show Graph")
    if show_animation:  # pragma: no cover
        plt_lib.draw(cx,cy, color = "ro", label="course")
        plt_lib.draw(states.x, states.y, color = "-b", label="trajectory")
        plt_lib.draw(cx[target_index], cy[target_index], color = "go", label="target")

        plt.subplots(1)

        # plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt_lib2 = PlotLib(xlabel="Time[s]", ylabel="Speed[km/h]")
        plt_lib2.draw(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt_lib2.show()

if __name__ == "__main__":
    main()
