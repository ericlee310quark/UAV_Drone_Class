import tello 
import cv2
import time

import os.path as path
import numpy as np

def main():
    drone = tello.Tello('', 8889)
    time.sleep(20)

    text = []
    for i in range(20):
        str1 = str(i)+'.jpg'
        text.append(str1)

    ind = 0

    while True:
        frame = drone.read()
        grey_f = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cv2.imshow("drone", frame)

        key = cv2.waitKey(1)

        if key!=-1:
            drone.keyboard(key)
            cv2.imwrite(text[ind], grey_f)
            ind+=1


    cv2.destoryAllWindows()
if __name__ == '__main__':
    main()