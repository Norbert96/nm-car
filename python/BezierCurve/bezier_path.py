from BezierCurve.bezier import Bezier
import numpy as np


class BezierPath():
    def __init__(self, continous=True):
        self.bcurves = []
        self.first_point = None
        self.continous = continous
        self.closed = False

    def add_point(self, point):
        point = np.array(point)
        if self.first_point is None:
            self.first_point = point
            return
        if self.bcurves:
            last_point_in_path = self.bcurves[-1][3]
            last_control_point_in_path = self.bcurves[-1][2]
            dist = np.linalg.norm(last_point_in_path - point)
            dist /= 3
            direction = (last_point_in_path - last_control_point_in_path)
            direction /= np.linalg.norm(last_point_in_path - point)

            new_control_point = last_point_in_path + direction * dist
            # dist/=3
            curve_points = np.array([last_point_in_path, new_control_point, point - np.array([0, 1]) * dist, point])
            self.bcurves.append(Bezier(curve_points[:, 0], curve_points[:, 1]))
        else:
            dist = np.linalg.norm(self.first_point - point)
            dist /= 3
            curve_points = np.array([self.first_point, self.first_point + np.array([1, 0]) * dist, point - np.array([0, -1]) * dist, point])
            self.bcurves.append(Bezier(curve_points[:, 0], curve_points[:, 1]))

    def json_serialize(self):
        data = {}
        data['bezier_curves'] = []

        for b in self.bcurves:
            curve = {}
            x = list(b.x)
            y = list(b.y)
            curve['x'] = x
            curve['y'] = y
            data['bezier_curves'].append(curve)

        return data

    def look_for_point(self, x, y):
        mouse_coord = np.array([x, y])
        for b in self.bcurves:
            i = 0
            for p in b.get_points():
                if np.linalg.norm(p - mouse_coord) < 4:
                    return (b, i)
                i += 1
        return False

    def calc_direction_vector(self, p1, p2):
        return (p1 - p2) / np.sqrt((p1 - p2).dot(p1 - p2))

    def distance_of_points(self, p1, p2):
        return np.linalg.norm(p1 - p2)

    def make_it_continous(self, point_ident):
        bezier = point_ident[0]
        n = point_ident[1]
        index = self.bcurves.index(bezier)
        if n == 1:
            point_to_modify = self.bcurves[index - 1][2]
            middle_point = self.bcurves[index - 1][3]
            dist = self.distance_of_points(point_to_modify, middle_point)
            self.bcurves[index - 1][2] = middle_point + dist * self.calc_direction_vector(middle_point, bezier[n])
        if n == 2:
            try:
                point_to_modify = self.bcurves[index + 1][1]
                middle_point = self.bcurves[index + 1][0]
            except IndexError as e:
                if not self.closed:
                    raise e
                else:
                    point_to_modify = self.bcurves[0][1]
                    middle_point = self.bcurves[0][0]
                    dist = self.distance_of_points(point_to_modify, middle_point)
                    self.bcurves[0][1] = middle_point + dist * self.calc_direction_vector(middle_point,
                                                                                          bezier[n])
                    return

            dist = self.distance_of_points(point_to_modify, middle_point)
            # print(self.calc_direction_vector( middle_point, bezier[n]))
            self.bcurves[index + 1][1] = middle_point + dist * self.calc_direction_vector(middle_point, bezier[n])

    def close_if_should(self, point_ident, x, y):
        bezier = point_ident[0]
        n = point_ident[1]

        index = self.bcurves.index(bezier)

        if [index, n] == [0, 0]:
            if self.distance_of_points([x, y], self.bcurves[-1][3]) < 4:
                print('closed')
                self.closed = True
                self.move_point((bezier, 1), bezier[1][0], bezier[1][1])
                bezier[n] = self.bcurves[-1][3]
        if [index, n] == [len(self.bcurves) - 1, 3]:
            if self.distance_of_points([x, y], self.bcurves[0][0]) < 4:
                self.closed = True
                print('closed')
                self.move_point((bezier, 2), bezier[1][0], bezier[1][1])
                a = self.bcurves[0][0]
                bezier[n] = self.bcurves[0][0]

    def move_point(self, point_ident, x, y):
        bezier = point_ident[0]
        n = point_ident[1]
        offset = [x, y] - bezier[n]
        bezier[n] = [x, y]

        index = self.bcurves.index(bezier)
        last_index = len(self.bcurves) - 1

        if [index, n] in [[0, 1], [0, 0], [last_index, 2], [last_index, 3]] and not self.closed:
            if n == 0:
                bezier[1] += offset
            if n == 3:
                bezier[2] += offset

            return

        if n == 0:
            bezier[1] += offset
            next_bezier = self.bcurves[index - 1]

            next_bezier[3] = [x, y]
            next_bezier[2] += offset
        if n == 3:
            bezier[2] += offset
            try:
                next_bezier = self.bcurves[index + 1]

            except IndexError as e:
                if not self.closed:
                    raise e
                else:
                    next_bezier = self.bcurves[0]

            next_bezier[0] = [x, y]
            next_bezier[1] += offset

        if self.continous and n in [1, 2]:
            self.make_it_continous(point_ident)


# path = BezierPath()


# for i in range(100):
#   path.add_point(np.random.randint(100, size=2))

# # path.add_point([5,5])


# import numpy as np
# import matplotlib.pyplot as plt


# for i in path.bcurves[:]:
#   i.print_nodes()
#   x, y = i.get_bezier_points(90)

#   plt.plot(x,y)

# plt.show()


# print(path.bcurves)
