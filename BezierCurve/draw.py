from BezierCurve.bezier import Bezier
import cv2


def get_bezier_points(bezier, scale):
    x = []
    y = []
    for i in range(scale + 1):
        t = (i / scale)
        p = bezier.get(t)
        x.append(p[0])
        y.append(p[1])
    return x, y


def draw_line_from_distance(frame, bezier, dist, color=(0, 255, 255), size=2):
    x, y = bezier.get_bezier_points_offseted(dist)
    for i in range(len(x) - 1):
        cv2.line(frame, (int(x[i]), int(y[i])), (int(x[i + 1]), int(y[i + 1])), color, size)


def draw_scaled_bezier(frame, bezier, offset):
    for i in bezier.reduce():
        for mult in [1, -1]:
            scaled_bezier = i.scale(mult * offset)
            draw_bezier(frame, scaled_bezier, (0, 0, 255))


def draw_bezier(frame, bezier, color=(0, 0, 255), size=2, scale=20):
    x, y = bezier.get_bezier_points(scale)
    for i in range(len(x) - 1):
        cv2.line(frame, (int(x[i]), int(y[i])), (int(x[i + 1]), int(y[i + 1])), color, size)


def draw_control_points(frame, bezier, color, helper_line=True):
    points = bezier.get_bezier_control_points(bezier)
    if helper_line:
        cv2.line(frame, points[1], points[0], (255, 255, 255), 1)
    cv2.circle(frame, points[1], 4, color, -1)
    if helper_line:
        cv2.line(frame, points[2], points[3], (255, 255, 255), 1)
    cv2.circle(frame, points[2], 4, color, -1)


def draw_points(frame, bezier):
    cv2.circle(frame, (int(bezier[0][0]), int(bezier[0][1])), 4, (0, 0, 255), -1)
    cv2.circle(frame, (int(bezier[3][0]), int(bezier[3][1])), 4, (0, 0, 255), -1)
