import math
import numpy as np

class State:

    def __init__(self, Wheel_Base, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.wb = Wheel_Base
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v 
        self.rear_x = self.x - ((self.wb / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wb / 2) * math.sin(self.yaw))

    def update(self, a, delta, dt):
        self.x += self.v * math.cos(self.yaw)  
        self.y += self.v * math.sin(self.yaw) 
        self.yaw += self.v / self.wb * math.tan(delta) 
        self.v += a * dt
        self.rear_x = self.x - ((self.wb / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wb / 2) * math.sin(self.yaw))

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

    def __init__(self, cx, cy, Ld, Lf_k):
        self.cx = cx
        self.cy = cy
        self.Ld = Ld
        self.Lf_k = Lf_k
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
                try:
                    distance_next_index = state.calc_distance(self.cx[index + 1], self.cy[index + 1])
                    if distance_this_index < distance_next_index:
                        break
                    index = index + 1 if (index + 1) < len(self.cx) else index
                    distance_this_index = distance_next_index
                    self.old_nearest_point_index = index
                except IndexError as e:
                    print(e)
                    return

        Lf_d = self.Lf_k * state.v + self.Ld

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
        try:
            delta = math.atan2(2.0 * state.wb * math.sin(alpha) / Ld , 1.0)
        except:
            delta = 0
        return delta, target_index

    @staticmethod
    def rad2deg (radian):
        return radian * 180.0 / math.pi

    @staticmethod
    def deg2rad(degree) :
	    return degree * math.pi / 180.0