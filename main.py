from RaceTrack.map import Map
import numpy as np
import cv2

m = Map('racetrack-1555403390.json')
print(m.curve_list)


while True:
    frame = np.zeros((800, 1000, 3), np.uint8)
    m.draw_map(frame)

    # for p in points:

    #     cv2.circle(frame, p,4,  (0,255,0),-1)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1)

    if key == 27:
        break


cv2.destroyAllWindows()
