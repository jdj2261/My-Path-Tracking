import numpy as np

class State:
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, Wheel_Base, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.wb = Wheel_Base
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v 

    def update(self, acceleration, delta, dt):
        self.x += self.v * np.cos(self.yaw)  
        self.y += self.v * np.sin(self.yaw) 
        self.yaw += self.v / self.wb * np.tan(delta) 
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt

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
    def __init__(self):
        pass

    @staticmethod
    def search_target_index(state, cx, cy):
        """
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        front_x = state.x + state.wb * np.cos(state.yaw)
        front_y = state.y + state.wb * np.sin(state.yaw)

        dx = [front_x - icx for icx in cx]
        dy = [front_y - icy for icy in cy]
        distance = np.hypot(dx, dy)

        min_distance = min(distance)
        target_index = np.argmin(distance)

        """
        I dont understande
        """
        return target_index, min_distance

class Stanley:
    def __init__(self):
        pass

    @staticmethod
    def stanley_control(state, cx, cy, cyaw, last_target_index):
        pass
    
def rad2deg (radian):
    return radian * 180.0 / np.pi

def deg2rad(degree) :
    return degree * np.pi / 180.0

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle