import numpy as np
import math


class Bezier():
    def __init__(self, x, y, start=0.0, end=1.0):
        assert x.size == y.size, 'dim of x, y should be equal'
        self.x = x
        self.y = y
        self.t1 = start
        self.t2 = end
        self.order = self.x.size - 1
        self.dpoint_x = None
        self.dpoint_y = None
        self.clockwise = None
        self.stacked_points = None
        self.reduced = []
        self.update()

    def update(self):
        self.dpoint_x, self.dpoint_y = self.derive(self.x, self.y)
        self.clockwise = self.compute_direction()
        self.stacked_points = np.stack((self.x, self.y), axis=-1)
        self.reduced = []

    def get(self, t):
        mt = 1 - t
        mt2 = mt**2
        t2 = t * t

        a = mt2 * mt
        b = mt2 * t * 3
        c = mt * t2 * 3
        d = t * t2

        abcd = np.array([a, b, c, d])
        x = np.inner(abcd, self.x)  # sum of product, a*x1 + b*x2....
        y = np.inner(abcd, self.y)
        return np.array([x, y])

    def __getitem__(self, key):
        return np.array([self.x[key], self.y[key]])

    def __setitem__(self, key, value):
        self.x[key] = value[0]
        self.y[key] = value[1]
        self.update()

    def get_points(self):
        p = np.stack((self.x, self.y), axis=-1)
        return p

    def get_derivative(self, t):
        mt = 1 - t
        a = mt * mt
        b = mt * t * 2
        c = t * t
        dx = self.dpoint_x[0]
        dy = self.dpoint_y[0]
        abc = np.array([a, b, c])
        x = np.inner(abc, dx)
        y = np.inner(abc, dy)
        return [x, y]

    def compute_direction(self):
        p = np.stack((self.x, self.y), axis=-1)
        angle = self.angle(p[0], p[self.order], p[1])
        return angle > 0

    def print_nodes(self):
        p = np.stack((self.x, self.y), axis=-1)
        print(p)

    def derive(self, points_x, points_y):
        dpoint_x = []
        dpoint_y = []
        order = points_x.size - 1

        p_x = points_x
        p_y = points_y

        d = points_x.size
        c = d - 1
        while d > 1:
            l_x = []
            l_y = []
            for j in range(c):
                x = c * (p_x[j + 1] - p_x[j])
                y = c * (p_y[j + 1] - p_y[j])
                l_x.append(x)
                l_y.append(y)

            d -= 1
            c -= 1
            dpoint_x.append(l_x)
            dpoint_y.append(l_y)
            p_x = l_x
            p_y = l_y
        return (dpoint_x, dpoint_y)

    def normal(self, t):
        d = self.get_derivative(t)
        q = math.sqrt(d[0] * d[0] + d[1] * d[1])
        if q < 0.001 and q > -0.001:
            print("will be zero")
        ret = [-d[1] / q, d[0] / q]
        return np.array(ret)

    def droots(self, p):

        if p.size == 3:
            a = p[0]
            b = p[1]
            c = p[2]
            d = a - 2 * b + c
            if d != 0:
                if 0 > b * b - a * c:
                    return []
                m1 = -math.sqrt(b * b - a * c)
                m2 = -a + b
                v1 = -(m1 + m2) / d
                v2 = -(-m1 + m2) / d
                return [v1, v2]
            elif b != c and d == 0:
                return [(2 * b - c) / (2 * (b - c))]
            else:
                return []
        if p.size == 2:
            a = p[0]
            b = p[1]
            if a != b:
                return [a / (a - b)]
            return []

    def extrema(self):
        x_val = []
        y_val = []
        for i in range(2):
            p_x = self.dpoint_x[i]
            p_y = self.dpoint_y[i]
            x_val += self.droots(np.array(p_x))
            y_val += self.droots(np.array(p_y))
        x_val = list(filter(lambda x: x >= 0 and x <= 1, x_val))
        y_val = list(filter(lambda x: x >= 0 and x <= 1, y_val))

        return x_val, y_val

    def lerp(self, r, v1, v2):
        p = v1 + r * (v2 - v1)
        return p

    def hull(self, t):
        p = self.stacked_points
        q = p
        while p.shape[0] > 1:
            _p = np.empty((0, 2), float)

            for i in range(p.shape[0] - 1):
                pt = self.lerp(t, p[i], p[i + 1])

                _p = np.append(_p, np.array([pt]), axis=0)

                q = np.append(q, np.array([pt]), axis=0)
            p = _p

        return q

    def map(self, v, ds, de, ts, te):
        d1 = de - ds
        d2 = te - ts
        v2 = v - ds
        r = v2 / d1
        return ts + d2 * r

    def split(self, t1, t2=None):
        # TODO: make a shortcut for t1==0 or t2 == 1
        q = self.hull(t1)

        q_x = q[:, 0]
        q_y = q[:, 1]
        indices_left = [0, 4, 7, 9]
        indices_right = [9, 8, 6, 3]

        result_left = Bezier(np.take(q_x, indices_left), np.take(q_y, indices_left))
        result_right = Bezier(np.take(q_x, indices_right), np.take(q_y, indices_right))

        result_left.t1 = self.map(0, 0, 1, self.t1, self.t2)
        result_left.t2 = self.map(t1, 0, 1, self.t1, self.t2)
        result_right.t1 = self.map(t1, 0, 1, self.t1, self.t2)
        result_right.t2 = self.map(1, 0, 1, self.t1, self.t2)

        if t2 == None:
            return (result_left, result_right)

        t2 = self.map(t2, t1, 1, 0, 1)
        subsplit = result_right.split(t2)

        # if subsplit[0]*4 == [i for i in subsplit[i]]:
        #   print('equal')
        #   print('equal')

        return subsplit[0]

    def angle(self, o, v1, v2):
        dx1 = v1[0] - o[0]
        dy1 = v1[1] - o[1]
        dx2 = v2[0] - o[0]
        dy2 = v2[1] - o[1]
        cross = dx1 * dy2 - dy1 * dx2
        dot = dx1 * dx2 + dy1 * dy2
        return math.atan2(cross, dot)

    def simple(self):

        p = np.stack((self.x, self.y), axis=-1)
        a1 = self.angle(p[0], p[3], p[1])
        a2 = self.angle(p[0], p[3], p[2])
        if (a1 > 0 and a2 < 0) or (a2 > 0 and a1 < 0):
            return False

        n1 = self.normal(0)
        n2 = self.normal(1)

        s = n1[0] * n2[0] + n1[1] * n2[1]
        angle = abs(math.acos(s))
        return angle < math.pi / 3

    def reduce(self):
        if self.reduced:
            return self.reduced
        t1 = 0
        t2 = 0
        step = 0.1
        first = []
        second = []
        x, y = self.extrema()

        extremas = x + y
        extremas.sort()

        if 0 not in extremas:
            extremas = [0] + extremas
        if 1 not in extremas:
            extremas.append(1)

        t1 = extremas[0]
        for e in extremas[1:]:
            t2 = e
            segment = self.split(t1, t2)
            segment.t1 = t1
            segment.t2 = t2
            first.append(segment)
            t1 = t2

        for p1 in first:
            t1 = 0
            t2 = 0

            while t2 <= 1:
                t2 = t1 + step
                while t2 <= 1:
                    segment = p1.split(t1, t2)
                    if not segment.simple():
                        t2 -= step
                        if abs(t1 - t2) < step:
                            return []
                        segment = p1.split(t1, t2)
                        segment.t1 = self.map(t1, 0, 1, p1.t1, p1.t2)
                        segment.t2 = self.map(t2, 0, 1, p1.t1, p1.t2)
                        second.append(segment)
                        t1 = t2
                        break

                    t2 += step
            if t1 < 1:
                segment = p1.split(t1, 1)
                segment.t1 = self.map(t1, 0, 1, p1.t1, p1.t2)
                segment.t2 = p1.t2
                second.append(segment)
        self.reduced = second
        return self.reduced

    def offset(self, t, d):
        # TODO linear
        c = self.get(t)
        n = self.normal(t)
        x = c[0] + n[0] * d
        y = c[1] + n[1] * d
        return [x, y]

    def lli4(self, p1, p2, p3, p4):
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]
        x3 = p3[0]
        y3 = p3[1]
        x4 = p4[0]
        y4 = p4[1]

        nx = (x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)
        ny = (x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)
        d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        # TODO: an exception would be better here
        if d == 0:
            return False
        return np.array([nx / d, ny / d])

    def scale(self, d):
        self.order
        clockwise = self.clockwise

        v = [self.offset(0, 10), self.offset(1, 10)]
        points = ap = np.stack((self.x, self.y), axis=-1)

        ap[0] = ap[0] + d * np.array(self.normal(0))
        ap[-1] = ap[-1] + d * np.array(self.normal(1))

        c0 = np.array(self.get(0))
        c1 = np.array(self.get(1))
        n0 = np.array(self.normal(0))
        n1 = np.array(self.normal(1))

        o = self.lli4(c0 + n0 * 10, c0, c1 + n1 * 10, c1)

        for t in range(2):
            p = ap[t * self.order]
            d = np.array(self.get_derivative(t))
            p2 = p + d
            ap[t + 1] = self.lli4(p, p2, o, points[t + 1])
        return Bezier(ap[:, 0], ap[:, 1])

    def outline(self, d1):
        reduced = self.reduce()
        fcurves = []
        bcurves = []
        for segment in reduced:
            fcurves.append(segment.scale(d1))
            bcurves.append(segment.scale(-d1))

    def get_bezier_points(self, scale):
        x = []
        y = []
        for i in range(scale + 1):
            t = (i / scale)
            p = self.get(t)
            x.append(p[0])
            y.append(p[1])
        return x, y

    def get_bezier_points_offseted(self, offset, scale=20):
        x = []
        y = []
        for i in range(scale + 1):
            t = i / scale
            p = self.get(t)
            n = self.normal(t)
            o = p + n * offset
            x.append(o[0])
            y.append(o[1])
        return x, y

    def get_bezier_control_points(self, bezier, integer=True):
        ret = []
        for i in range(4):
            if integer:
                ret.append((int(bezier[i][0]), int(bezier[i][1])))
            else:
                ret.append((bezier[i][0], bezier[i][1]))
        return ret
