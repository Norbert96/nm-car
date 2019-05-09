from BezierCurve.bezier import Bezier
import json
import numpy as np
from BezierCurve.draw import *


class Map():
    def __init__(self, map_file):

        self.curve_list = self.load_map(map_file)
        self.current_distance = 0
        self.map_distance_intervals = self.create_distance_interval()
        self.path_len = self.map_distance_intervals[-1][1]

    def draw_map(self, frame):
        return self.draw_map_line(frame)

    def draw_map_line(self, frame):
        for bezier in self.curve_list:
            draw_bezier(frame, bezier)
        return frame

    def draw_road(self, frame):
        m = np.zeros(frame.shape)

    def create_distance_interval(self):
        # intervals = np.array()
        intervals = []
        for i in range(len(self.curve_list)):
            length = self.curve_list[i].get_approximate_lenght()
            if not len(intervals):
                interval = [0, length]
            else:

                interval = [intervals[-1][1], intervals[-1][1] + length]
            intervals.append(interval)
        return np.array(intervals)

    def load_map(self, map_file):
        with open(map_file) as json_file:
            map_dict = json.load(json_file)
            curves = []
            for curve in map_dict['bezier_curves']:
                x = curve['x']
                y = curve['y']
                bezier = Bezier(np.array(x), np.array(y))
                curves.append(bezier)
            return curves

    def get_point_distance_from_start(self, distance_from_start):
        distance_from_start = distance_from_start % self.path_len
        b = ((self.map_distance_intervals[:, 0] <= distance_from_start) & (self.map_distance_intervals[:, 1] >= distance_from_start))
        indexes = np.argwhere(b).reshape(-1)
        idx = indexes[0]
        curve = self.curve_list[idx]
        curve_start_point = self.map_distance_intervals[idx][0]
        distance_on_curve = distance_from_start - curve_start_point
        point = curve.get(distance_on_curve / curve.get_approximate_lenght())
        return point

    def forward(self, m):
        # m = self.relative_path_len_ratio * m
        self.current_distance += m
        return self.get_point_distance_from_start(self.current_distance)

    def get_rabbit_position(self):
        return self.get_point_distance_from_start(self.current_distance)

    def restart(self):
        self.current_distance = 0
