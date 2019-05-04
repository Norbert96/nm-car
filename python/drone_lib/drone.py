
import numpy as np
from .drone_consts import *


class Drone(object):
    def __init__(self):
        self.yaw_ang = 0
        self.pitch_ang = 0
        self.roll_ang = 0
        self.target_state = np.zeros((3,))
        self.current_state = np.zeros((3,))
        self.controll_to_rad_transfer = np.array([max_pitch_angle, max_roll_angle, 1])
        self.change = np.array([max_d_pitch, max_d_roll, max_d_yaw])
        self.gravity_force = 9.80665 * mass
        self.speed = np.array([0, 0])

    def controll(self, cnts):
        self.target_state = self.controll_to_rad(cnts)

    def sim(self, time):
        self.forward(time)
        pull_force = self.gravity_force * np.tan(self.current_state[:2])
        pull_force = self.rotate(pull_force, self.current_state[2])

        Fd = self.calculate_air_resistance()
        force = pull_force + Fd
        acc = force / mass
        self.speed = self.speed + acc * time
        print('pull_force: {} Fd: {} speed: {} acc: {} '.format(pull_force, Fd, self.speed, acc))
        return acc

    def calculate_air_resistance(self):
        speedabs = np.linalg.norm(self.speed)
        if speedabs == 0:
            return np.array([0,0])
        Fdabs = -0.5 * speedabs ** 2 * density_of_air * cd * A
        Fd = (self.speed/speedabs)*Fdabs
        return Fd

    def rotate(self, vec, theta):
        c, s = np.cos(theta), np.sin(theta)
        r_matrix = np.array([[c, -s], [s, c]])
        return r_matrix.dot(vec)

    def get_direction(self):
        return self.current_state[2]

    def forward(self, time):
        step_change = time * self.change
        diff = self.target_state - self.current_state
        if np.abs(diff[0]) < np.abs(step_change[0]):
            self.current_state[0] = self.target_state[0]
        else:
            self.current_state[0] += step_change[0] * (1 if diff[0] > 0 else -1)

        if np.abs(diff[1]) < np.abs(step_change[1]):
            self.current_state[1] = self.target_state[1]
        else:
            self.current_state[1] += step_change[1] * (1 if diff[1] > 0 else -1)

        self.current_state[2] += step_change[2] * self.target_state[2]
        self.normalize_rotation()

    def normalize_rotation(self):
        if self.current_state[2] < 0:
            self.current_state[2] = 2 * np.pi + self.current_state[2]

        if self.current_state[2] > 2 * np.pi:
            self.current_state[2] = self.current_state - 2 * np.pi

    def controll_to_rad(self, cnts):
        return self.controll_to_rad_transfer * cnts


# def try_drone():

#     d = Drone()
#     controll = [1, 1, 0]
#     d.controll(controll)

#     for i in range(100):
#         if i > 20:
#             d.controll([1,0,0])
#         print(d.get_acc(0.01))


# try_drone()
