from ClassicControlling.classic_controll import ClassicControll
from RaceTrack.map import Map
from BezierCurve.draw import *
import numpy as np
from drone_lib.drone import Drone
import cv2
import time


class OptimalizationControllAgent(ClassicControll):
    def __init__(self, map_json, max_distance_from_middle=20):
        ClassicControll.__init__(self, map_json)
        self.simulation_time = 0.1
        self.speed_section_num = 100
        self.initial_speed = 8
        self.speed_delta = 1
        self.modifying_section_num = 10
        self.max_distance_from_middle = max_distance_from_middle
        self.map = OptimalizationMap(map_json, self.max_distance_from_middle)
        self.round = 0
        self.i = 0
        self.rabbit_speed = 15
        self.last_modified_sections = []
        self.out_start_zone = False
        self.section_speed = []

        self.first_track = None
        self.last_track = None


    def start_optimalization(self):
        self.section_speed = [self.initial_speed] * self.speed_section_num
        try:
            while True:

                self.res()
                succes, time, track = self.one_round()
                if self.first_track is None:
                    self.first_track = track
                if succes and time > 10:

                    print("LAP time: {:.1f} s".format(time))
                    self.increase_speed_randomly()
                    self.last_track = track
                else:
                    print("Failed")
                    self.decrease_last_modifed_section()
        except KeyboardInterrupt:
            print("interrupted")
        finally:
            self.create_jpg_of_track()


    def increase_speed_randomly(self):
        self.last_modified_sections = []
        for i in range(self.modifying_section_num):
            speed_sect_to_modify = np.random.randint(self.speed_section_num)
            self.section_speed[speed_sect_to_modify] += self.speed_delta
            self.last_modified_sections.append(speed_sect_to_modify)

    def decrease_last_modifed_section(self):
        if len(self.last_modified_sections):
            for sect in self.last_modified_sections:
                self.section_speed[sect] -= self.speed_delta
        self.last_modified_sections = []


    def create_jpg_of_track(self):
        frame = np.zeros((800, 1000, 3), np.uint8)
        self.map.draw_map(frame, draw_middle_line=False)
        all_speed = [i[2] for i in self.first_track] + [i[2] for i in self.last_track]
        min_speed = min(all_speed)
        max_speed = max(all_speed)
        self.draw_track_line(frame, self.first_track, min_speed, max_speed)
        self.draw_track_line(frame, self.last_track, min_speed, max_speed)
        cv2.imwrite('first_and_last_track.png', frame)





    def draw_track_line(self,frame, track, min_speed, max_speed):
        if track is None:
            return

        def calculate_color_num(speed):
            delta = 255/(max_speed-min_speed)
            return delta*(speed-min_speed)

        for i in range(len(track) - 1):
            p_s = track[i]
            p_s1 = track[i +1]

            color_num = calculate_color_num(p_s[2])
            frame = cv2.line(frame, (int(p_s[0]), int(p_s[1])), (int(p_s1[0]), int(p_s1[1])), (0, 255-color_num, color_num))




    def one_round(self):
        time = 0
        track = []
        while True:
            speed_sect = self.map.in_which_speed_section(self.speed_section_num)
            self.rabbit_speed = self.section_speed[speed_sect]
            track.append(self.run(self.simulation_time))
            time += self.simulation_time
            if not self.map.point_on_route(self.drone.position):
                return False, time, track
            if self.is_finished():
                return True, time, track

    def res(self):
        self.out_start_zone = False
        self.drone = Drone()
        self.drone.position = self.map.get_rabbit_position()
        self.map.restart()

    def run(self, t):
        self.drone.controll(self.calculate_controlling_parameters())
        self.drone.sim(t)
        self.map.forward(self.rabbit_speed * t)
        self.i += 1
        drone_position = self.drone.position
        drone_speed = np.linalg.norm(self.drone.speed)
        return [drone_position[0], drone_position[1], drone_speed]
        # if self.round < self.map.current_distance // self.map.path_len:
        #     print(self.map.current_distance // self.map.path_len)
        #     self.round = self.map.current_distance // self.map.path_len
        #     print(self.i)

    def is_finished(self):
        distance_from_finish_point = self.map.distance_from_finish_point(self.drone.position)
        if distance_from_finish_point > self.max_distance_from_middle:
            self.out_start_zone = True

        if self.out_start_zone and self.max_distance_from_middle > distance_from_finish_point:
            return True

        return False


class OptimalizationMap(Map):
    def __init__(self, map_file, distance_from_midline):
        Map.__init__(self, map_file)
        self.max_distance_from_midline = distance_from_midline
        self.path_line_points = self.create_line_points()
        self.middle_line_points = self.create_middle_line_points()
        self.finish_line = self.create_finish_line()

    def draw_map(self, frame, draw_middle_line=True):
        if draw_middle_line:
            frame = self.draw_map_line(frame)
        for bezier in self.curve_list:
            draw_line_from_distance(frame, bezier, self.max_distance_from_midline)
            draw_line_from_distance(frame, bezier, -self.max_distance_from_midline)

    def create_finish_line(self):
        x1, y1 = self.curve_list[0].get_bezier_points_offseted(10)
        x2, y2 = self.curve_list[0].get_bezier_points_offseted(-10)
        line = np.array([[x1[0], y1[0]], [x2[0], y2[0]]])
        return line

    def distance_from_finish_point(self, drone_position):
        starting_point = self.curve_list[0].get(0)
        distance_from_finish_point = np.linalg.norm(drone_position - starting_point)

        return distance_from_finish_point

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
            cv2.line(frame, p1, p2, (255, 255, 255), 1)

        return frame

    def line_distance_from_point(self, l1, l2, p):
        min = 100000000
        a = 0
        for i in range(len(l1)):
            p1 = l1[i]
            p2 = l2[i]

            d = self.DistancePointLine(p1, p2, p)

            if d < min:
                min = d
            #cross = np.cross(p2 - p1, p - p1)
            #cross = np.linalg.norm(cross)
            #norm = np.linalg.norm(p2 - p1)

            # if (cross / norm) < min:
            #    min = np.abs(cross/norm)
            #    a = i

        print(a)

        return min

    def lineMagnitude(self, x1, y1, x2, y2):
        import math
        lineMagnitude = np.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
        return lineMagnitude

    def DistancePointLine(self, p1, p2, p):
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
        closest_dist2 = np.linalg.norm(self.middle_line_points - point, axis=1).min()
        #print('By line: ' + str(closest_dist1))
        #print('By point: ' + str(closest_dist2))
        return closest_dist2 < self.max_distance_from_midline

    def in_which_speed_section(self, section_number):
        dist_from_start = self.current_distance % self.path_len
        section_index = dist_from_start // (self.path_len / section_number)
        return int(section_index)

    # def step(self, time):
