from BezierCurve.bezier import Bezier
import json
import numpy as np
from BezierCurve.draw import *


class Map():
    def __init__(self, map_file):

        self.curve_list = self.load_map(map_file)

    def draw_map_line(self, frame):
        for bezier in self.curve_list:
            draw_bezier(frame, bezier)

    def draw_road(self, frame):
        m = np.zeros(frame.shape)

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
