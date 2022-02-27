import os.path as path
import numpy as np
import cv2

if __name__ == '__main__':
    img = cv2.imread(path.join('lisa.jpeg'))
    
    height, width = img.shape[:2]

    
    img2 = np.zeros((3*height, 3*width, 3), dtype=np.uint8)
    height_n, width_n = img2.shape[:2]

    print(height, width)

    print(height_n, width_n)

    for y in range(3*height-1):
        for x in range(3*width-1):




            Blue = img[int(y/3),int(x/3),0]* (int(y/3)+1-float(y/3)) * (int(x/3)+1-float(x/3))
            Blue += img[int(y/3)+1,int(x/3),0]* (float(y/3)-int(y/3)) * (int(x/3)+1-float(x/3))
            Blue += img[int(y/3),int(x/3)+1,0]* (int(y/3+1)-float(y/3)) * (float(x/3)-int(x/3))
            Blue += img[int(y/3)+1,int(x/3)+1,0]* (float(y/3)-int(y/3)) * (float(x/3)-int(x/3))

            Green = img[int(y/3),int(x/3),1]* (int(y/3)+1-float(y/3)) * (int(x/3)+1-float(x/3))
            Green += img[int(y/3)+1,int(x/3),1]* (float(y/3)-int(y/3)) * (int(x/3)+1-float(x/3))
            Green += img[int(y/3),int(x/3),1]* (int(y/3+1)-float(y/3)) * (float(x/3)-int(x/3))
            Green += img[int(y/3)+1,int(x/3),1]* (float(y/3)-int(y/3)) * (float(x/3)-int(x/3))

            Red = img[int(y/3),int(x/3),2]* (int(y/3)+1-float(y/3)) * (int(x/3)+1-float(x/3))
            Red += img[int(y/3)+1,int(x/3),2]* (float(y/3)-int(y/3)) * (int(x/3)+1-float(x/3))
            Red += img[int(y/3),int(x/3),2]* (int(y/3+1)-float(y/3)) * (float(x/3)-int(x/3))
            Red += img[int(y/3)+1,int(x/3),2]* (float(y/3)-int(y/3)) * (float(x/3)-int(x/3))

    cv2.imshow('new img', img)
    cv2.imshow('new img2', img2)
    cv2.waitKey()
    cv2.destroyAllWindows()
'''
            img2[y,x,0] = Blue
            img2[y,x,1] = Green
            img2[y,x,2] = Red


            y_select_1 = 0
            y_select_2 = 0
            x_select_1 = 0
            x_select_2 = 0
            

            if(y%3==0):
                y_select_1 = y/3
            elif(y%3 ==1):
                y_select_1 = (y-1)/3
                y_select_2 = y_select_1 + 1
            else:    
                y_select_2 = (y+1)/3
                y_select_1 = y_select_2 - 1

            if(x%3==0):
                x_select_1 = x/3
            elif(x%3 ==1):
                x_select_1 = (x-1)/3
                x_select_2 = x_select_1 + 1
            else:
                x_select_2 = (x+1)/3
                x_select_1 = x_select_2 - 1
            

            Up_left = (x-(x_select_1*3))/((x_select_2-x_select_1)*3)
            Up_right = ((x_select_2*3)-x)/((x_select_2-x_select_1)*3)
            
            pixel_1 = Up_right*(img[y_select_1,x_select_1,:])+Up_left*(img[y_select_1,x_select_2,:])
            
            
            y_select = int(y_select)
            x_select = int(x_select)
 '''           #print("y_select:{}\nx_select{}".format(y_select,x_select))

            


            
    