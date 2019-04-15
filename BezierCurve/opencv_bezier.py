import cv2
import numpy as np


from bezier_path import BezierPath

from draw_helper import *


class RoadDesigner:

    def __init__(self):
        self.bpath = BezierPath()
        self.drag = False
        self.moving_point = None

    def draw(self, frame):
        i = 0
        offset = 30
        for b in self.bpath.bcurves:
            color = (255, 0, 0)
            if i % 2 == 0:
                color = (0, 255, 0)
            i += 1

            self.draw_bezier(frame, b, color)
            self.draw_scaled_bezier(frame, b, offset)
            self.draw_control_points(frame, b, color)
            self.draw_points(frame, b)

    def draw_scaled_bezier(self, frame, bezier, offset):
        for i in bezier.reduce():
            for mult in [1, -1]:
                scaled_bezier = i.scale(mult * offset)
                self.draw_bezier(frame, scaled_bezier, (0, 0, 255))

    def draw_bezier(self, img, bezier, color, scale=20):
        x, y = bezier.get_bezier_points(scale)
        for i in range(len(x) - 1):
            cv2.line(img, (int(x[i]), int(y[i])), (int(x[i + 1]), int(y[i + 1])), color, 2)

    def draw_control_points(self, img, bezier, color, helper_line=True):
        points = get_bezier_points(bezier)
        cv2.line(img, points[1], points[0], (255, 255, 255), 1)
        cv2.circle(frame, points[1], 4, color, -1)
        cv2.line(img, points[2], points[3], (255, 255, 255), 1)
        cv2.circle(frame, points[2], 4, color, -1)

    def draw_points(self, img, bezier):

        cv2.circle(frame, (int(bezier[0][0]), int(bezier[0][1])), 4, (0, 0, 255), -1)
        cv2.circle(frame, (int(bezier[3][0]), int(bezier[3][1])), 4, (0, 0, 255), -1)

    def delete_last_point(self):
        self.bpath.bcurves = self.bpath.bcurves[:-1]
        self.bpath.closed = False
        if len(self.bpath.bcurves) == 0:
            self.bpath = BezierPath()

    def reduce_curves(self):
        reduced_curves = []
        for c in self.bpath.bcurves:
            reduced_curves += c.reduce()
        self.bpath.bcurves = reduced_curves

    def mouse(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            point_ident = self.bpath.look_for_point(x, y)

            if point_ident:
                self.moving_point = point_ident
                self.drag = True
        if event == cv2.EVENT_MOUSEMOVE:
            if self.drag:
                self.bpath.move_point(self.moving_point, x, y)

        if event == cv2.EVENT_LBUTTONUP:
            if self.drag:  # dropping
                self.drag = False
                self.bpath.close_if_should(self.moving_point, x, y)
                return
            if not self.bpath.closed:
                self.bpath.add_point([x, y])

    def key_pressed(self, key):
        if key == ord("d"):
            self.delete_last_point()
        if key == ord('r'):
            self.reduce_curves()



# cap = cv2.VideoCapture(0)
np.seterr(all='warn')
road_designer = RoadDesigner()
cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", road_designer.mouse)
points = []


while True:
    frame = np.zeros((800, 1000, 3), np.uint8)
    road_designer.draw(frame)

    # for p in points:

    #     cv2.circle(frame, p,4,  (0,255,0),-1)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)

    if key == 27:
        break
    else:
        road_designer.key_pressed(key)
cap.release()
cv2.destroyAllWindows()
