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
        self.dpoint_x, self.dpoint_y = self.derive(self.x, self.y)

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
        return x, y

    def get_derivative(self, t):
        mt = 1 - t
        a = mt * mt
        b = mt * t * 2
        c = t * t
        dx  = self.dpoint_x[0]
        dy = self.dpoint_y[0]
        abc = np.array([a, b, c])
        x = np.inner(abc, dx)
        y = np.inner(abc, dy)
        return [x, y]

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
        return [-d[1] / q, d[0] / q]

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
        p_x = self.x[:]
        p_y = self.y[:]

        p = np.stack((p_x, p_y), axis=-1)

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
        t1 = 0
        t2 = 0
        step = 0.01
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
                while t2 <= 1 + step:
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

        return second


x = np.array([100, 10, 110, 150])
y = np.array([25, 90, 100, 195])

b = Bezier(x, y)
print(b.get(0.5))
print(b.get_derivative(0.5))
print(b.dpoint_x)
print(b.extrema())
reduce = b.reduce()
curves = b.split(0.5)
curves[0].extrema()