
"""
@author: Gen
This is for Zipline Data Scientist Project
"""
import csv
import pandas
import matplotlib.pyplot as plt
import os
import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from time import time, sleep


#Read Data
script_dir = os.path.dirname(__file__)  # Script directory
flight_path = os.path.join(script_dir, 'data_scientist_take-home\\')

df = pandas.read_csv(flight_path + 'flight_16951.csv')

#Assign data:
#X is North or South
#Y is East or West
#Z is Up or Down
time = df['seconds_since_launch']
posX = df['position_ned_m[0]']
posY = df['position_ned_m[1]']
posZ = df['position_ned_m[2]']
velX = df['velocity_ned_mps[0]']
velY = df['velocity_ned_mps[1]']
velZ = df['velocity_ned_mps[0]']
accX = df['accel_body_mps2[0]']
accY = df['accel_body_mps2[1]']
accZ = df['accel_body_mps2[2]']
roll = df['orientation_rad[0]']
pitch = df['orientation_rad[1]']
yaw = df['orientation_rad[2]']
angVelX = df['angular_rate_body_radps[0]']
angVelY = df['angular_rate_body_radps[1]']
angVelZ = df['angular_rate_body_radps[2]']
sigmaX = df['position_sigma_ned_m[0]']
sigmaY = df['position_sigma_ned_m[1]']
sigmaZ = df['position_sigma_ned_m[2]']

#Plotting
#plt.plot(time, accX)
timeDif = []
for i in range(len(time) - 1):
    timeDif.append(time[i + 1] - time[i])
timeCount = 0
#time.sleep(1)

fig = plt.figure()
ax = p3.Axes3D(fig)

ax.set_xlim3d([min(posX), max(posX)])
ax.set_xlabel('X')

ax.set_ylim3d([min(posY), max(posY)])
ax.set_ylabel('Y')

ax.set_zlim3d([-1 * max(posZ), -1 * min(posZ)])
ax.set_zlabel('Z')

def gen(n):
    for i in range(len(posX)):
        yield np.array([posX[i], posY[i], -1 * posZ[i]])

def update(num, data, line):
    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])
    global timeCount
    sleep(timeDif[timeCount])
    timeCount += 1

N = 1000
data = np.array(list(gen(N))).T
line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1])

ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=10000/N, blit=False)
#ani.save('matplot003.gif', writer='imagemagick')
plt.show()









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


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
     
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

