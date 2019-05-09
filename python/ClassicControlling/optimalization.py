from ClassicControlling.classic_controll import ClassicControll
from RaceTrack.map import Map
from BezierCurve.draw import *
import numpy as np


class OptimalizationControllAgent(ClassicControll):
    def __init__(self, map_json):
        ClassicControll.__init__(self, map_json)
        self.speed_section_num = 100
        self.initial_speed = 5
        self.speed_delta = 1
        self.map = OptimalizationMap(map_json, 10)
        self.round = 0
        self.i = 0
        self.rabbit_speed = 15

    def start_optimalization(self):
        self.section_speed = [self.initial_speed] * self.speed_section_num

        while True:
            self.run(0.1)

    def run(self, t):
        self.drone.controll(self.calculate_controlling_parameters())
        self.drone.sim(t)
        self.map.forward(self.rabbit_speed * t)
        self.i += 1
        # if self.round < self.map.current_distance // self.map.path_len:
        #     print(self.map.current_distance // self.map.path_len)
        #     self.round = self.map.current_distance // self.map.path_len
        #     print(self.i)

        self.map.point_on_route(self.drone.position)


class OptimalizationMap(Map):
    def __init__(self, map_file, distance_from_midline):
        Map.__init__(self, map_file)
        self.max_distance_from_midline = distance_from_midline
        self.path_line_points = self.create_line_points()

    def draw_map(self, frame):
        frame = self.draw_map_line(frame)
        for bezier in self.curve_list:
            draw_line_from_distance(frame, bezier, self.max_distance_from_midline)
            draw_line_from_distance(frame, bezier, -self.max_distance_from_midline)

    def create_line_points(self, resolution=100):
        delta = self.path_len / resolution
        p1 = []
        p2 = []
        for i in range(resolution):
            p1.append(self.get_point_distance_from_start(delta * i))
            p2.append(self.get_point_distance_from_start(delta * (i + 1)))

        p1 = np.array(p1)
        p2 = np.array(p2)
        return np.array([p1, p2])

    def draw_path_line_points(self, frame):
        import cv2
        for i in range(len(self.path_line_points[0])):
            p1 = tuple(self.path_line_points[0][i].astype(int))
            p2 = tuple(self.path_line_points[1][i].astype(int))
            cv2.line(frame,p1 , p2, (255, 255, 255), 1)

        return  frame


    def line_distance_from_point(self, l1, l2, p):
        min = 100000000
        a = 0
        for i in range(len(l1)):
            p1 = l1[i]
            p2 = l2[i]

            d = self.DistancePointLine(p1,p2,p)

            if d < min:
                min = d
            #cross = np.cross(p2 - p1, p - p1)
            #cross = np.linalg.norm(cross)
            #norm = np.linalg.norm(p2 - p1)

            #if (cross / norm) < min:
            #    min = np.abs(cross/norm)
            #    a = i


        print(a)


        return min

    def lineMagnitude(self,x1, y1, x2, y2):
        import math
        lineMagnitude = np.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
        return lineMagnitude

    def DistancePointLine(self,p1,p2,p):
        # http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba

        px = p[0]
        py = p[1]

        x1 = p1[0]
        y1 = p1[1]

        x2 = p2[0]
        y2 = p2[1]

        LineMag = self.lineMagnitude(x1, y1, x2, y2)

        if LineMag < 0.00000001:
            DistancePointLine = 9999
            return DistancePointLine

        u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
        u = u1 / (LineMag * LineMag)

        if (u < 0.00001) or (u > 1):
            # // closest point does not fall within the line segment, take the shorter distance
            # // to an endpoint
            ix = self.lineMagnitude(px, py, x1, y1)
            iy = self.lineMagnitude(px, py, x2, y2)
            if ix > iy:
                DistancePointLine = iy
            else:
                DistancePointLine = ix
        else:
            # Intersecting point is on the line, use the formula
            ix = x1 + u * (x2 - x1)
            iy = y1 + u * (y2 - y1)
            DistancePointLine = self.lineMagnitude(px, py, ix, iy)

        return DistancePointLine

    def point_on_route(self, point):
        closest_dist = self.line_distance_from_point(self.path_line_points[0], self.path_line_points[1], point).min()
        print(closest_dist)
        return closest_dist < self.max_distance_from_midline

    def in_which_speed_section(self, section_number):
        dist_from_start = self.current_distance % self.path_len
        section_index = dist_from_start // (path_len / section_number)
        return section_index

    # def step(self, time):
