from ClassicControlling.classic_controll import ClassicControll
from RaceTrack.map import Map
from BezierCurve.draw import *
import numpy as np


class OptimalizationControllAgent(ClassicControll):
    def __init__(self, map_json,max_distance_from_middle=20):
        ClassicControll.__init__(self, map_json)
        self.speed_section_num = 100
        self.initial_speed = 5
        self.speed_delta = 1
        self.max_distance_from_middle = max_distance_from_middle
        self.map = OptimalizationMap(map_json,self.max_distance_from_middle )
        self.round = 0
        self.i = 0
        self.rabbit_speed = 15

        self.out_start_zone = False
        self.last_distance_from_finish_line = None

    def start_optimalization(self):
        self.section_speed = [self.initial_speed] * self.speed_section_num

        while True:
            self.run(0.1)

    def res(self):
        self.out_start_zone = False
        self.last_distance_from_finish_line = None


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
        print(self.is_finished())


    def is_finished(self):
        distance_from_finish_line, distance_from_finish_point = self.map.distance_from_finish_line_and_point(self.drone.position)
        if distance_from_finish_point > self.max_distance_from_middle:
            self.out_start_zone = True

        if self.out_start_zone:
            if self.max_distance_from_middle > distance_from_finish_line: #in start zone again
                if self.last_distance_from_finish_line is not None:
                    if self.last_distance_from_finish_line * distance_from_finish_line < 0.0:
                        return True
                self.last_distance_from_finish_line = distance_from_finish_line

        return False








class OptimalizationMap(Map):
    def __init__(self, map_file, distance_from_midline):
        Map.__init__(self, map_file)
        self.max_distance_from_midline = distance_from_midline
        self.path_line_points = self.create_line_points()
        self.middle_line_points = self.create_middle_line_points()
        self.finish_line = self.create_finish_line()

    def draw_map(self, frame):
        frame = self.draw_map_line(frame)
        for bezier in self.curve_list:
            draw_line_from_distance(frame, bezier, self.max_distance_from_midline)
            draw_line_from_distance(frame, bezier, -self.max_distance_from_midline)

    def create_finish_line(self):
        x1, y1 = self.curve_list[0].get_bezier_points_offseted(10)
        x2, y2 = self.curve_list[0].get_bezier_points_offseted(-10)
        line = np.array([[x1[0],y1[0]], [x2[0], y2[0]]])
        return line

    def distance_from_finish_line_and_point(self, drone_position):
        p1 = self.finish_line[0]
        p2 = self.finish_line[1]
        p3 = drone_position

        distance_from_finish_line = np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1)
        starting_point = self.curve_list[0].get(0)
        distance_from_finish_point = np.linalg.norm(drone_position - starting_point)

        return distance_from_finish_line, distance_from_finish_point




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

    def create_middle_line_points(self, resolution=1000):
        delta = self.path_len / resolution
        p = []
        for i in range(resolution):
            p.append(self.get_point_distance_from_start(delta * i))
        return np.array(p)


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
        #closest_dist1 = self.line_distance_from_point(self.path_line_points[0], self.path_line_points[1], point).min()
        closest_dist2 = np.linalg.norm(self.middle_line_points-point, axis=1).min()
        #print('By line: ' + str(closest_dist1))
        print('By point: ' + str(closest_dist2))
        return closest_dist2 < self.max_distance_from_midline

    def in_which_speed_section(self, section_number):
        dist_from_start = self.current_distance % self.path_len
        section_index = dist_from_start // (path_len / section_number)
        return section_index

    # def step(self, time):
