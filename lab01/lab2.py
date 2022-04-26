import os.path as path
import numpy as np
import cv2
SCALE = 3
def NN_interpolation(img, dstH, dstW):
    scrH,scrW,_=img.shape
    retimg=np.zeros((dstH, dstW, 3), dtype=np.uint8)
    for y in range(dstH-1):
        for x in range(dstW-1):
            select_x = round(x*(scrW/dstW))
            select_y = round(y*(scrH/dstH))
            retimg[y, x] = img[select_y, select_x]
    return retimg


if __name__ == '__main__':
    img = cv2.imread(path.join('lisa.jpeg'))
    
    height, width = img.shape[:2]
    img2 = NN_interpolation(img,img.shape[0]*SCALE, img.shape[1]*SCALE)
    height_n, width_n = img2.shape[:2]

    print(height, width)

    print(height_n, width_n)

    cv2.imshow('Original_Lisa', img)
    cv2.imshow('NN_interpolation', img2)
    cv2.waitKey()
    cv2.destroyAllWindows()