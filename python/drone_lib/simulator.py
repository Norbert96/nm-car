from .drone import Drone
import numpy as np


class DroneRTSimulator(object):
    def __init__(self, drone, simulation_freq=100):
        self.drone = drone
        self.simulation_period = 1 / simulation_freq
        self.position = np.array([0, 0])
        self.speed = np.array([0, 0])

    def controll(self, cnt):
        self.drone.controll(cnt)

    def set_starting_position(self, starting_position):
        self.position = starting_position

    def simulate(self, sim_time = None):
        if sim_time:
            self.drone.sim(sim_time)
        else:
            self.drone.sim(self.simulation_period)
        self.position = self.position + self.drone.speed * self.simulation_period


# def try_drone():

#     d = Drone()
#     controll = [1, 1, 0]
#     d.controll(controll)
#     ds = DroneRTSimulator(d)
#     for i in range(100):
#         if i > 20:
#             ds.controll([1, 0, 0])
#         ds.simulate()
#         print("Speed: {}".format(ds.speed))
#         print("Position: {}".format(ds.position))


# try_drone()
