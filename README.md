"# Zip"

Code in Python
Install: pandas, matplotlib, numpy

For Mac or some other cases: in simulation.py line 20 and plot.py line 20
change '\\' to '/'

#Two important files:
1. plot.py
2. simulation.py
To run: type python "filename" in command line

#plot.py
-Allows you to choose flight number and dependent and independent variables and makes the graphs for you! Graph is saved in the figures folder and is shown when the code is run.

#simulation.py
-Gives 3D visualization of the flight of your choice! The speed and acceleration are incorporated. The drone's roll, pitch, and yaw motion are also added. You can see the video in the files in case you have trouble running the file.

1. Findings
-There were flights which were stable and ones that experienced turbulence or unsteady control.
-There were discontinuities in some flight paths maybe due to little errors in position measurements.

2. Outliers
-Flight 17128 and 17130 had low launch ground speed (27.5 to 28) compared to other data (29 to 31).
-High wind magnitude of about 7.5 for flight 17162 compared to others (0 to 5).

3. Weather
-Air temp. (16.5 - 35)
-Relative humidity (35 - 74)
-Static Pressuer (80000 - 80900)
-Wind Direction (majority are negative)
