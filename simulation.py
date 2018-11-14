
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
import math
import sys


#Read Data
script_dir = os.path.dirname(__file__)  # Script directory
flight_path = os.path.join(script_dir, 'data_scientist_take-home\\')

#ask user for flight number
flightNum = str(input("Enter a valid flight number (16951 to 17745) : "))
try:
    df = pandas.read_csv(flight_path + "flight_" + flightNum + ".csv")
except Exception as e:
    print(flight_path + "flight_" + flightNum + ".csv" + " does not exist")
    sys.exit()
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
velZ = df['velocity_ned_mps[2]']
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

#this is the drone
planeX = [1, 0.8, 0.6, 0.4, 0.2, 0, -0.2, -0.4, -0.6, -0.8, -1, -1, -1, -1, -1, -1, -0.1, -0.2, -0.3, -0.4, -0.5, -0.1, -0.2, -0.3, -0.4, -0.5, -1, -1, -1, -1, -1, -1]
planeY = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.1, -0.2, -0.3, -0.4, -0.5, 0.1, 0.2, 0.3, 0.4, 0.5, -0.1, -0.2, -0.3, 0.1, 0.2, 0.3]
planeZ = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02, 0.04, 0.06, 0.08, 0.10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
planeSize = 30
for i in range(len(planeX)):
    planeX[i] *= planeSize
    planeY[i] *= planeSize
    planeZ[i] *= planeSize
plane3D = np.array([planeX, planeY, planeZ])

#this will be used for orientation calculation
temp = [[0 for x in range(len(planeX))] for y in range(3)]
temp2 = [[0 for x in range(len(planeX))] for y in range(3)]

#this will be used to make read data into correct format
def gen():
    for i in range(len(posX)):
        yield np.array([posX[i], posY[i], -1 * posZ[i]])

#input list of eulerian angles in radians
#output rotation matrix
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

#called upon animating the drone flight motion
def update(num, data, line):

    line.set_data(data[:2, :num])
    line.set_3d_properties(data[2, :num])
    currOrientation = [roll[timeCount], -1 * pitch[timeCount], yaw[timeCount]]
    currR = eulerAnglesToRotationMatrix(currOrientation)
    temp = np.matmul(currR, plane3D)
    #orientation of the plane
    for i in range(3):
        for j in range(len(planeX)):
            temp2[i][j] = temp[i][j] + data[i][timeCount]

    ax.clear()
    ax.set_xlim3d([min(posX), max(posX)])
    ax.set_xlabel('X')

    ax.set_ylim3d([min(posY), max(posY)])
    ax.set_ylabel('Y')

    ax.set_zlim3d([-1 * max(posZ), -1 * min(posZ)])
    ax.set_zlabel('Z')
    ax.plot(temp2[0][:], temp2[1][:], temp2[2][:], 'ro')

    global timeCount
    sleep(0.1 * timeDif[timeCount])
    if timeCount == len(timeDif) - 1:
        plt.close()
        sys.exit()
    timeCount += 1

N = 1000
data = np.array(list(gen())).T
line, = ax.plot(data[0, 0:1], data[1, 0:1], data[2, 0:1])

ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=10000/N, blit=False)

plt.show()
# plt.close()
