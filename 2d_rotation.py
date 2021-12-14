import numpy as np
import matplotlib.pyplot as plt
import math

#Code for 2D rotation
#The four vectors form a square
#Angle of rotation is given as input by user
#Blue-origial square, Orange-Rotated

#defining the vectors
v1 = np.array([[0],
              [0]])
v2 = np.array([[5],
              [0]])
v3 = np.array([[5],
              [5]])
v4 = np.array([[0],
              [5]])
xpoints = np.array([v1[0][0],v2[0][0],v3[0][0],v4[0][0],v1[0][0]])
ypoints = np.array([v1[1][0],v2[1][0],v3[1][0],v4[1][0],v1[1][0]])
plt.plot(xpoints,ypoints)

angle = float(input("Enter angle= "))
theta = math.pi*angle/180
c = math.cos(theta)
s = math.sin(theta)

#rotation matrix
A =np.array([[c,-s],
             [s,c]])
print("rotation matrix=",A)

v1_new = np.dot(A,v1)
v2_new = np.dot(A,v2)
v3_new = np.dot(A,v3)
v4_new = np.dot(A,v4)
plt.axis("scaled")
plt.xlim([-10,10])
plt.ylim([-10,10])
xpoints = np.array([v1_new[0][0],v2_new[0][0],v3_new[0][0],v4_new[0][0],v1_new[0][0]])
ypoints = np.array([v1_new[1][0],v2_new[1][0],v3_new[1][0],v4_new[1][0],v1_new[1][0]])
plt.plot(xpoints,ypoints)
plt.show()

