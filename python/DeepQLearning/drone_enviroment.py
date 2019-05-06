from drone_lib.drone import Drone
import numpy as np


class DroneEnv():
    def __init__(self):
        self.drone = None
        self.time = 1 / 20
        self.rabbit_speed = 5
        self.rabbit_line = np.array([[0, 0], [10000, 0]])
        self.rabbit_start = 0
        self.last_distance_from_rabbit = None
        self.done()

    def get_rabbit_position(self):
        # done = False

        dist = np.linalg.norm(self.rabbit_line)
        pos = self.rabbit_line[0] * (1 - self.rabbit_start) + self.rabbit_line[1] * self.rabbit_start

        self.rabbit_start += (self.time / dist) * self.rabbit_speed
        return pos
        # if self.rabbit_start > 1:
        #     done = True

    def calculate_reward(self, distance_from_rabbit):
        reward = 100 - (2 * distance_from_rabbit)**4
        return reward

    def calculate_reward_by_pos_diff(self, distance_from_rabbit):
        reward = self.last_distance_from_rabbit - distance_from_rabbit
        if reward > -1 and reward < 1:
            reward = 1 if reward < 0 else -1
        reward = int(reward*10)
        return reward

    def done(self):
        self.rabbit_start = 0
        self.drone = Drone()
        self.drone.speed = np.array([5,0])
        self.last_distance_from_rabbit = 0


    def reset(self):
        self.done()
        state = np.array([self.drone.speed, self.drone.get_direction_vector(),[0, 0]])
        return state

    def step(self, action):
        done = False
        self.drone.controll(action)
        num = 5
        for i in range(num):
            self.drone.sim(self.time/num)
        position = self.get_rabbit_position()
        position_difference = position - self.drone.position

        next_state = np.array([self.drone.speed,  self.drone.get_direction_vector(), position_difference])
        distance_from_rabbit = np.linalg.norm(position_difference)
        reward = self.calculate_reward_by_pos_diff(distance_from_rabbit)
        if distance_from_rabbit > 10:
            done = True
            self.done()

        return next_state, reward, done
