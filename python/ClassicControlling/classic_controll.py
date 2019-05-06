

from drone_lib.drone import Drone
from RaceTrack.map import Map
import numpy as np


class ClassicControll():
    def __init__(self, map_json):
        self.drone = Drone()
        self.drone.current_state[2] = 0
        self.map = Map(map_json)
        self.rabbit_speed = 20
        self.reset()

    def run(self, t):
        self.drone.controll(self.calculate_controlling_parameters())
        self.drone.sim(t)
        self.map.forward(self.rabbit_speed * t)

    def reset(self):
        self.drone.position = self.map.get_rabbit_position()

    def rotate(self, vec, drone_dir):
        c, s = np.cos(drone_dir), np.sin(drone_dir)
        r_matrix = np.array([[c, -s], [s, c]])
        return r_matrix.dot(vec)

    def calculate_controlling_parameters(self):
        drone_to_rabbit_vector = self.map.get_rabbit_position() - self.drone.position
        drone_to_rabbit_vector = self.rotate(drone_to_rabbit_vector, self.drone.get_direction())
        pitch_distance = drone_to_rabbit_vector[0]

        drone_direction  = self.drone.get_direction()
        drone_to_rabbit_direction = np.arctan2(drone_to_rabbit_vector[0], drone_to_rabbit_vector[1])
        rad_diff = drone_to_rabbit_direction - drone_direction
        rad_diff = (rad_diff + np.pi/2) % np.pi - np.pi/2



        roll_distance = drone_to_rabbit_vector[1]
        controll = np.array([pitch_distance, roll_distance, 0])
        controll = controll / 100
        controll[2] *=100
        controll = np.tanh(controll)

        print(controll)
        return controll

    def get_drone_position(self):
        return self.drone.position

    def get_rabbit_position(self):
        return self.map.get_rabbit_position()
