
from ClassicControlling.optimalization import OptimalizationControllAgent
import numpy as np
import cv2
import time


def rotate(vec, theta):
    c, s = np.cos(theta), np.sin(theta)
    r_matrix = np.array([[c, -s], [s, c]])
    return r_matrix.dot(vec)


def draw_drone(frame, pos, dir):
    l2 = np.array([10, 0])
    l2 = rotate(l2, dir)
    l2 = pos + l2
    l2 = l2.astype(int)
    l1 = pos.astype(int)
    pos = pos.astype(int)
    frame = cv2.line(frame, (l1[0], l1[1]), (l2[0], l2[1]), (0, 255, 255), 3)
    frame = cv2.circle(frame, (pos[0], pos[1]), 5, (0, 255, 0), -1)
    return frame


map_file = 'racetrack-1555403390.json'

cc = OptimalizationControllAgent(map_file)
cc.start_optimalization()
start_time = time.time()
while True:
    frame = np.zeros((800, 1000, 3), np.uint8)
    cc.map.draw_map(frame)

    frame = cc.map.draw_path_line_points(frame)
    #cc.run(time.time() - start_time)
    cc.run(0.1)

    start_time = time.time()
    rabbit_position = cc.get_rabbit_position().astype(int)
    drone_position = cc.get_drone_position().astype(int)

    cv2.circle(frame, (rabbit_position[0], rabbit_position[1]), 4, (255, 0, 0), -1)
    draw_drone(frame, cc.drone.position, cc.drone.get_direction())
    # for p in points:

    #     cv2.circle(frame, p,4,  (0,255,0),-1)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)

    if key == 27:
        break


cv2.destroyAllWindows()
