import cv2
import drone_lib as dlib
import numpy as np
import time


refresh_freq = 100
title_window = "DroneSimulator"
pitch = 0
roll = 0
yaw = 0
cv2.namedWindow(title_window)
frame_size = np.array([500, 500])

d = dlib.Drone()
ds = dlib.DroneRTSimulator(d,refresh_freq)


def slider_value_to_control(value):
    return (value - 5)/5.0


def pitch_slider(value):
    global pitch
    pitch = slider_value_to_control(value)


def roll_slider(value):
    global roll
    roll = slider_value_to_control(value)


def yaw_slider(value):
    global yaw
    yaw  = slider_value_to_control(value)


cv2.namedWindow(title_window)
cv2.createTrackbar("Pitch", title_window, 5, 10, pitch_slider)
cv2.createTrackbar("Roll", title_window, 5, 10, roll_slider)
cv2.createTrackbar("Yaw", title_window, 5, 10, yaw_slider)


def rotate(vec, theta):
    c, s = np.cos(theta), np.sin(theta)
    r_matrix = np.array([[c, -s], [s, c]])
    return r_matrix.dot(vec)


def draw_drone(frame, pos, dir):
    l2 = np.array([10,0])
    l2 = rotate(l2, dir)
    l2 = pos + l2
    l2 = l2.astype(int)
    l1 = pos
    frame = cv2.line(frame, (l1[0], l1[1]), (l2[0] , l2[1] ), (0,0,255))
    frame = cv2.circle(frame, (pos[0], pos[1]), 5, (255, 0, 0), -1)
    return  frame

path_list = []

def draw_path(frame, position):
    global path_list
    if not len(path_list):
        path_list.append(position)
    if not np.array_equal(path_list[-1], position):
        path_list.append(position.astype(int))

    for i in range(len(path_list)-1):
        p1 = path_list[i]
        p2 = path_list[i + 1]
        frame = cv2.line(frame, (p1[0], p1[1]), (p2[0], p2[1]), (0, 0, 255))
    return frame

time_start = time.time()
while True:
    frame = np.zeros((frame_size[0], frame_size[1], 3))
    control = np.array([pitch, roll, yaw])
    ds.controll(control)
    ds.simulate(time.time()-time_start)
    time_start = time.time()
    pos = ds.position
    # print(pos)
    #print(ds.drone.speed)
    pos = pos + frame_size / 2  # offset to center
    pos = pos.astype(int)
    frame = draw_drone(frame, pos, d.get_direction())
    frame = draw_path(frame, pos)

    cv2.imshow(title_window, frame)

    cv2.waitKey(1)
    #time.sleep(1 / refresh_freq)



cv2.waitKey()
