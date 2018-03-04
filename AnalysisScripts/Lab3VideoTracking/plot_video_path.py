"""
Given two files output from tracker for the same run and the corresponding 
odometry, plot the path the robot took.
"""
import matplotlib
# print(matplotlib.get_backend())
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import linregress

PATH_HEAD = "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/LabData/"


RIGHT_DOT_FILEPATH  = PATH_HEAD+"red_dot.csv"
LEFT_DOT_FILEPATH   = PATH_HEAD+"blue_dot.csv"
# desired_tick_rate_L desired_tick_rate_R state_est_x state_est_y state_est_theta deltaleftEnc deltarightEnc
ODOMETRY_FILEPATH   = PATH_HEAD+"pointTrackingHARD_FINAL_18-03-03 10.40.38_LargeThreshold_1_1.8_neg1.csv"
# Add later
# SIMULATION_FILEPATH = ""

CONVERT_M_TO_CM = 100

# Each adjacent corner is 30.5 cm apart
CORNER_DIFFERENCE_CM = 30.5
# dots on robot are 10cm apart
DOT_DIFFERENCE_CM = 10.0

NW='NW'
N='N'
NE='NE'
W='W'
C='C'
E='E'
SW='SW'
S='S'
SE='SE'

ref_points = {
  'NW': np.array([-157.6, 100.1]),
  'N': np.array([-54.46, 97.87]),
  'NE': np.array([47.17, 96.67]),
  'W': np.array([-159.6, -2.451]),
  'C': np.array([-56.60, -3.79]),
  'E': np.array([44.65, -5.46]),
  'SW': np.array([-162.2, -105.8]),
  'S': np.array([-59.28, -106.7]),
  'SE': np.array([44.67, -109.0]),
}

# data taken from frames of the video file and tracker
ideal_points = {
  'NW': np.array([-31.5, 31.5]),
  'N': np.array([0, 31.5]),
  'NE': np.array([31.5, 31.5]),
  'W': np.array([-31.5, 0]),
  'C': np.array([0,0]),
  'E': np.array([31.5, 0]),
  'SW': np.array([-31.5, -31.5]),
  'S': np.array([0, 31.5]),
  'SE': np.array([31.5, -31.5]),
}

tile_corners = ['NW', 'N', 'NE', 'W', 'C', 'E', 'SW', 'S', 'SE']
translated_points = []

C0x = ref_points['C'][0]
C0y = ref_points['C'][1]
C0px = np.array([C0x, C0y])
for pt in tile_corners:
  x, y = ref_points[pt]
  x -= C0x
  y -= C0y
  ref_points[pt][0] = x
  ref_points[pt][1] = y
# print(tile_corners)

# find rotation of point
rows = [[NW, N, NE], [W, C, E], [SW, S, SE]]
cols = [[NW, W, SW], [N, C, S], [NE, E, SE]]


# rotate points to orthogonal axes relative to lines on ground
# linreg_info = {}

# for pts in rows:
#   arr = np.zeros((len(pts),2))
#   ideal_arr = np.zeros((len(pts),2))
#   for i in range(len(pts)):
#     arr[i,:] = ref_points[pts[i]]
#     # ideal_arr[i,:] = ideal_points[pts[i]]
#     # slope, intercept, r_value, p_value, std_err
#     linreg_info['r'+str(i)] = linregress(arr[:,0], arr[:,1])

# for pts in cols:
#   arr = np.zeros((len(pts),2))
#   ideal_arr = np.zeros((len(pts),2))
#   for i in range(len(pts)):
#     arr[i,:] = ref_points[pts[i]]
#     # ideal_arr[i,:] = ideal_points[pts[i]]
#     # slope, intercept, r_value, p_value, std_err
#     linreg_info['c'+str(i)] = linregress(arr[:,1], arr[:,0])




# find pixel to centimeters conversion
dists_between_pts = []
for vec in rows+cols:
  for i in range(len(vec)-1):
    dist = np.linalg.norm(ref_points[vec[i+1]] - ref_points[vec[i]])
    print(vec[i+1], vec[i], dist)
    # print(dist)
    dists_between_pts.append(dist)

# conversion from point to centimeters
dists_between_pts = np.array(dists_between_pts)
# print(dists_between_pts, np.mean(dists_between_pts), np.std(dists_between_pts))
CONVERT_PX_TO_CM = CORNER_DIFFERENCE_CM/np.mean(dists_between_pts)
print('conversion from px to cm is ', CONVERT_PX_TO_CM, 'px/cm')



# get data from files
right_dot = pd.read_csv(RIGHT_DOT_FILEPATH)
left_dot  = pd.read_csv(LEFT_DOT_FILEPATH)
odometry  = pd.read_csv(ODOMETRY_FILEPATH)

# get position in pixels and recenter w.r.t root
video_pos_px = (right_dot[['x','y']] + left_dot[['x','y']])/2
video_pos_px -= video_pos_px.ix[0,:]
video_pos_cm = CONVERT_PX_TO_CM * video_pos_px

Dy = right_dot['y'] - left_dot['y']
Dx = right_dot['x'] - left_dot['x']
video_heading = np.arctan2(Dy, Dx)

print(video_pos_cm)

plt.subplot(3,1,1)

plt.plot(CONVERT_M_TO_CM * odometry['state_est_x'], -CONVERT_M_TO_CM * odometry['state_est_y'])
plt.plot(video_pos_cm['x'], video_pos_cm['y'])
plt.legend()
plt.subplot(3,1,2)
plt.plot(video_heading)
plt.subplot(3,1,3)
plt.plot(-odometry['state_est_theta'])
plt.legend()
plt.show()

