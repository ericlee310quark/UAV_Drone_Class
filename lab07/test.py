import numpy as np

refFace =[]
refFace.append([0.0,14.0,0.0])

print(refFace)
x1,x2,y1,y2 = 1,2,3,4
face_point =[]

face_point.append([x1,y1])
face_point.append([x2,y1])
face_point.append([x2,y2])
face_point.append([x1,y2])


face_point2 = np.array(face_point,dtype=np.float64)

print(face_point2)