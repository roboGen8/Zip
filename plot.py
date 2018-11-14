
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
print("0 - time")
print("1 - posX\t2 - posY\t3 - posZ")
print("4 - velX\t5 - velY\t6 - velZ")
print("7 - accX\t8 - accY\t9 - accZ")
print("10 - roll\t11 - pitch\t12 - yawZ")
print("13 - angVelX\t14 - angVelY\t15 - angVelZ")
print("16 - sigmaX\t17 - sigmaY\t18 - sigmaZ")
xAxis = int(input("Enter number code for x-axis: "))

yAxis = int(input("Enter number code for y-axis: "))

hashX = dict()
hashY = dict()

time = df['seconds_since_launch']
hashX[0] = time
hashY[0] = 'seconds_since_launch'

posX = df['position_ned_m[0]']
hashX[1] = posX
hashY[1] = 'position_ned_m[0]'

posY = df['position_ned_m[1]']
hashX[2] = posY
hashY[2] = 'position_ned_m[1]'

posZ = df['position_ned_m[2]']
hashX[3] = posZ
hashY[3] = 'position_ned_m[2]'

velX = df['velocity_ned_mps[0]']
hashX[4] = velX
hashY[4] = 'velocity_ned_mps[0]'

velY = df['velocity_ned_mps[1]']
hashX[5] = velY
hashY[5] = 'velocity_ned_mps[1]'

velZ = df['velocity_ned_mps[2]']
hashX[6] = velZ
hashY[6] = 'velocity_ned_mps[2]'

accX = df['accel_body_mps2[0]']
hashX[7] = accX
hashY[7] = 'accel_body_mps2[0]'

accY = df['accel_body_mps2[1]']
hashX[8] = accY
hashY[8] = 'accel_body_mps2[1]'

accZ = df['accel_body_mps2[2]']
hashX[9] = accZ
hashY[9] = 'accel_body_mps2[2]'

roll = df['orientation_rad[0]']
hashX[10] = roll
hashY[10] = 'orientation_rad[0]'

pitch = df['orientation_rad[1]']
hashX[11] = pitch
hashY[11] = 'orientation_rad[1]'

yaw = df['orientation_rad[2]']
hashX[12] = yaw
hashY[12] = 'orientation_rad[2]'

angVelX = df['angular_rate_body_radps[0]']
hashX[13] = angVelX
hashY[13] = 'angular_rate_body_radps[0]'

angVelY = df['angular_rate_body_radps[1]']
hashX[14] = angVelY
hashY[14] = 'angular_rate_body_radps[1]'

angVelZ = df['angular_rate_body_radps[2]']
hashX[15] = angVelZ
hashY[15] = 'angular_rate_body_radps[2]'

sigmaX = df['position_sigma_ned_m[0]']
hashX[16] = sigmaX
hashY[16] = 'position_sigma_ned_m[0]'

sigmaY = df['position_sigma_ned_m[1]']
hashX[17] = sigmaY
hashY[17] = 'position_sigma_ned_m[1]'

sigmaZ = df['position_sigma_ned_m[2]']
hashX[18] = sigmaZ
hashY[18] = 'position_sigma_ned_m[2]'

def genX():
    for i in range(len(rawX)):
        yield np.array(rawX[i] * 1.0)
def genY():
    for i in range(len(rawY)):
        yield np.array(rawY[i] * 1.0)

rawX = hashX[xAxis]
rawY = hashX[yAxis]
labelX = hashY[xAxis]
labelY = hashY[yAxis]


dataX = np.array(list(genX())).T
dataY = np.array(list(genY())).T


fig = plt.figure()
line, = plt.plot(dataX, dataY)

plt.xlabel(labelX)
plt.ylabel(labelY)
plt.legend(loc='best')
titl = labelX + " vs " + labelY
plt.title(titl)
plt.show()
fig.savefig("figures/" + flightNum + " " + titl)
plt.close(fig)
