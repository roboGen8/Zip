import csv
import pandas
import matplotlib.pyplot as plt
import os
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from time import time, sleep
import math



# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R


#Figure for drone motion
fig2 = plt.figure(2)
ax2 = p3.Axes3D(fig2)

ax2.set_xlim3d([-2, 2])
ax2.set_xlabel('X')

ax2.set_ylim3d([-2, 2])
ax2.set_ylabel('Y')

ax2.set_zlim3d([-2, 2])
ax2.set_zlabel('Z')

planeX = [1, 0.8, 0.6, 0.4, 0.2, 0, -0.2, -0.4, -0.6, -0.8, -1, -1, -1, -1, -1, -1, -0.1, -0.2, -0.3, -0.4, -0.5, -0.1, -0.2, -0.3, -0.4, -0.5, -1, -1, -1, -1, -1, -1]
planeY = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.1, -0.2, -0.3, -0.4, -0.5, 0.1, 0.2, 0.3, 0.4, 0.5, -0.1, -0.2, -0.3, 0.1, 0.2, 0.3]
planeZ = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
plane3D = [planeX, planeY, planeZ]

#plt.plot(planeX, planeY, planeZ, 'ok')
for i in range(90):
    theta0 = i * 1.0 * math.pi / 180 
    R = eulerAnglesToRotationMatrix([theta0, 0, 0])
    plane3D = np.matmul(R, plane3D)
    plt.plot(plane3D[0][:], plane3D[1][:], plane3D[2][:], 'ro')
    sleep(0.5)
    plt.show()

#plt.gcf().clear()


