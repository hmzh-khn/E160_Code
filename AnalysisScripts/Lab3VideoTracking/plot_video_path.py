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
import math

PATH_HEAD = "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/LabData/"

def normalize_angle(theta):
  '''
  Returns an angle in range (-pi, pi]
  '''
  out_angle = theta % (2 * math.pi)
  greater_than_pi = (out_angle > math.pi).astype(int)
  return out_angle - 2 * math.pi * greater_than_pi


RIGHT_DOT_FILEPATH  = PATH_HEAD+"red_dot.csv"
LEFT_DOT_FILEPATH   = PATH_HEAD+"blue_dot.csv"
# desired_tick_rate_L desired_tick_rate_R state_est_x state_est_y state_est_theta deltaleftEnc deltarightEnc
ODOMETRY_FILEPATH   = PATH_HEAD+"pointTrackingHARD_FINAL_18-03-03 10.40.38_LargeThreshold_1_1.8_neg1.csv"
SIMULATION_FILEPATH = PATH_HEAD+"pointTrackingSOFT_FINAL_18-03-04 13.58.29_LargeThreshold_0.005_0.04_neg0.1.csv"

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
softsim   = pd.read_csv(SIMULATION_FILEPATH)

# flip reversed axes in odometry, software simulation
odometry['state_est_x'] = CONVERT_M_TO_CM * odometry['state_est_x']
odometry['state_est_y'] = CONVERT_M_TO_CM * odometry['state_est_y']
odometry['state_est_theta'] = odometry['state_est_theta']

softsim['state_est_x'] = CONVERT_M_TO_CM * softsim['state_est_x']
softsim['state_est_y'] = CONVERT_M_TO_CM * softsim['state_est_y']
softsim['state_est_theta'] = softsim['state_est_theta']

# get position in pixels and recenter w.r.t root
video_pos_px = (right_dot[['x','y']] + left_dot[['x','y']])/2
video_pos_px['y'] = -video_pos_px['y']
video_pos_px -= video_pos_px.ix[0,:]
video_pos_cm = CONVERT_PX_TO_CM * video_pos_px

Dy = right_dot['y'] - left_dot['y']
Dx = right_dot['x'] - left_dot['x']
video_heading = -np.arctan2(Dy, Dx)
video_heading -= video_heading[0]
video_heading = normalize_angle(video_heading)

video_state = pd.concat([video_pos_cm, video_heading], axis=1)

# plotting
stage1end = 929
stage2end = 1780
stage3end = 2156 # (2340)
stage4end = 2812
# print(video_pos_cm[0:stage1end], video_heading[0:stage1end])

plt.figure()
plt.title('full path - robot state (x, y) while tracking path')
plt.xlim(-30,30)
plt.ylim(-30,30)
plt.plot(video_pos_cm['x'], video_pos_cm['y'], label='video odometry')
plt.plot(odometry['state_est_x'], odometry['state_est_y'], label='hardware-in-the-loop simulation')


plt.plot(video_pos_cm['x'][0], video_pos_cm['y'][0], 'go')
plt.plot(video_pos_cm['x'][stage4end], video_pos_cm['y'][stage4end], 'rx')
# plt.plot(softsim['state_est_x'][0:118], softsim['state_est_y'][0:118], label='software simulation')
plt.xlabel('x pos (cm)')
plt.ylabel('y pos (cm)')
plt.legend()

plt.figure()
plt.title('full path software simulation - robot state (x, y) while tracking path')
plt.xlim(-30,30)
plt.ylim(-30,30)
plt.plot(softsim['state_est_x'], softsim['state_est_y'], label='software simulation')
plt.xlabel('x pos (cm)')
plt.ylabel('y pos (cm)')
plt.legend()


plt.figure()
plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0.25,0.25) -> (0,0,0)')
plt.subplot(4,1,1)
plt.xlim(-30,30)
plt.ylim(-30,30)
plt.plot(video_pos_cm[0:stage1end]['x'], video_pos_cm[0:stage1end]['y'], label='video odometry')
plt.plot(odometry['state_est_x'][0:159],odometry['state_est_y'][0:159], label='hardware-in-the-loop simulation')
plt.plot(softsim['state_est_x'][0:118], softsim['state_est_y'][0:118], label='software simulation')

plt.plot(video_pos_cm['x'][0], video_pos_cm['y'][0], 'go')
plt.plot(video_pos_cm['x'][stage1end], video_pos_cm['y'][stage1end], 'rx')
plt.ylabel('y pos (cm)')
plt.legend()
plt.subplot(4,1,2)
plt.ylim(-math.pi,math.pi)
plt.plot(video_heading[0:stage1end], label='video odometry')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,3)
plt.ylim(-math.pi,math.pi)
plt.plot(odometry['state_est_theta'][0:159], label='hardware-in-the-loop simulation')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,4)
plt.ylim(-math.pi,math.pi)
plt.plot(softsim['state_est_theta'][0:118], label='software simulation')
plt.ylabel('heading (rad)')
plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
plt.legend()

plt.figure()
plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0,0.25,0) -> (0,0,0)')
plt.subplot(4,1,1)
plt.xlim(-30,30)
plt.ylim(-30,30)
plt.plot(video_pos_cm[stage1end:stage2end]['x'], video_pos_cm[stage1end:stage2end]['y'], label='video odometry')
plt.plot(odometry['state_est_x'][159:309],odometry['state_est_y'][159:309], label='hardware-in-the-loop simulation')
plt.plot(softsim['state_est_x'][118:250], softsim['state_est_y'][118:250], label='software simulation')

plt.plot(video_pos_cm['x'][stage1end], video_pos_cm['y'][stage1end], 'go')
plt.plot(video_pos_cm['x'][stage2end], video_pos_cm['y'][stage2end], 'rx')
plt.ylabel('y pos (cm)')
plt.legend()
plt.subplot(4,1,2)
plt.plot(video_heading[stage1end:stage2end], label='video odometry')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,3)
plt.plot(odometry['state_est_theta'][159:309], label='hardware-in-the-loop simulation')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,4)
plt.ylim(-math.pi,math.pi)
plt.plot(softsim['state_est_theta'][118:250], label='software simulation')
plt.ylabel('heading (rad)')
plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
plt.legend()

plt.figure()
plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0,0,2.7) -> (0,0,-2.7) -> (0,0,2.7) -> (0,0,0)')
plt.subplot(4,1,1)
plt.xlim(-30,30)
plt.ylim(-30,30)
plt.plot(video_pos_cm[stage2end:stage3end]['x'], video_pos_cm[stage2end:stage3end]['y'], label='video odometry')
plt.plot(odometry['state_est_x'][309:374],odometry['state_est_y'][309:374], label='hardware-in-the-loop simulation')
plt.plot(softsim['state_est_x'][250:344], softsim['state_est_y'][250:344], label='software simulation')
plt.plot(video_pos_cm['x'][stage2end], video_pos_cm['y'][stage2end], 'go')
plt.plot(video_pos_cm['x'][stage3end], video_pos_cm['y'][stage3end], 'rx')
plt.ylabel('y pos (cm)')
plt.legend()
plt.subplot(4,1,2)
plt.plot(video_heading[stage2end:stage3end], label='video odometry')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,3)
plt.plot(odometry['state_est_theta'][309:374], label='hardware-in-the-loop simulation')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,4)
plt.ylim(-math.pi,math.pi)
plt.plot(softsim['state_est_theta'][250:344], label='software simulation')
plt.ylabel('heading (rad)')
plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
plt.legend()

fig = plt.figure()
plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0.25,0,pi) -> (0,0,pi)')
plt.subplot(4,1,1)
plt.xlim(-30,30)
plt.ylim(-30,30)
plt.plot(video_pos_cm[stage3end:stage4end]['x'], video_pos_cm[stage3end:stage4end]['y'], label='video odometry')
plt.plot(odometry['state_est_x'][374:500],odometry['state_est_y'][374:500], label='hardware-in-the-loop simulation')
plt.plot(softsim['state_est_x'][344:460], softsim['state_est_y'][344:460], label='software simulation')
plt.plot(video_pos_cm['x'][stage3end], video_pos_cm['y'][stage3end], 'go')
plt.plot(video_pos_cm['x'][stage4end], video_pos_cm['y'][stage4end], 'rx')
plt.ylabel('y pos (cm)')
plt.legend()
plt.subplot(4,1,2)
plt.plot(video_heading[stage3end:stage4end], label='video odometry')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,3)
plt.plot(odometry['state_est_theta'][374:500], label='hardware-in-the-loop simulation')
plt.ylabel('heading (rad)')
plt.legend()
plt.subplot(4,1,4)
plt.ylim(-math.pi,math.pi)
plt.plot(softsim['state_est_theta'][344:560], label='software simulation')
plt.ylabel('heading (rad)')
plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
plt.legend()



plt.show()

