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


# RIGHT_DOT_FILEPATH  = PATH_HEAD+"red_dot.csv"
# LEFT_DOT_FILEPATH   = PATH_HEAD+"blue_dot.csv"
# desired_tick_rate_L desired_tick_rate_R state_est_x state_est_y state_est_theta deltaleftEnc deltarightEnc
# ODOMETRY_FILEPATH   = PATH_HEAD+"KnownStart_P400_Hardware.csv"
SIMULATION_FILEPATH = PATH_HEAD+"KnownStart_P400_Simulation.csv"

CONVERT_M_TO_CM = 100

# get data from files
# hardsim = pd.read_csv(ODOMETRY_FILEPATH)
softsim = pd.read_csv(SIMULATION_FILEPATH)

# flip reversed axes in odometry, software simulation
# hardsim['state_est_x'] = CONVERT_M_TO_CM * hardsim['state_est_x']
# hardsim['state_est_y'] = CONVERT_M_TO_CM * hardsim['state_est_y']
# hardsim['state_est_theta'] = hardsim['state_est_theta']

softsim['pf_state_x'] = CONVERT_M_TO_CM * softsim['pf_state_x']
softsim['pf_state_y'] = CONVERT_M_TO_CM * softsim['pf_state_y']
softsim['pf_state_theta'] = softsim['pf_state_theta']

# hardsim['state_odo_x'] = CONVERT_M_TO_CM * hardsim['state_odo_x']
# hardsim['state_odo_y'] = CONVERT_M_TO_CM * hardsim['state_odo_y']
# hardsim['state_odo_theta'] = hardsim['state_odo_theta']

softsim['state_odo_x'] = CONVERT_M_TO_CM * softsim['state_odo_x']
softsim['state_odo_y'] = CONVERT_M_TO_CM * softsim['state_odo_y']
softsim['state_odo_theta'] = softsim['state_odo_theta']








plt.figure()
plt.title('full path state (x,y) - known start (400 particles)')
# plt.xlim(-30,30)
# plt.ylim(-30,30)
plt.plot(softsim['state_odo_x'], softsim['state_odo_y'], label='knownstart_sim_odo')
plt.plot(softsim['pf_state_x'], softsim['pf_state_y'], label='knownstart_sim_pf')
# plt.plot(, label='knownstart_hardware_odo')
# plt.plot(, label='knownstart_hardware_pf')

plt.xlabel('x pos (cm)')
plt.ylabel('y pos (cm)')
plt.legend()




# # plotting
# stage1end = 929
# stage2end = 1780
# stage3end = 2156 # (2340)
# stage4end = 2812
# # print(video_pos_cm[0:stage1end], video_heading[0:stage1end])

# plt.figure()
# plt.title('full path - robot state (x, y) while tracking path')
# plt.xlim(-30,30)
# plt.ylim(-30,30)
# plt.plot(video_pos_cm['x'], video_pos_cm['y'], label='video odometry')
# plt.plot(odometry['pf_state_x'], odometry['pf_state_y'], label='hardware-in-the-loop simulation')


# plt.plot(video_pos_cm['x'][0], video_pos_cm['y'][0], 'go')
# plt.plot(video_pos_cm['x'][stage4end], video_pos_cm['y'][stage4end], 'rx')
# # plt.plot(softsim['pf_state_x'][0:118], softsim['pf_state_y'][0:118], label='software simulation')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# plt.legend()

# plt.figure()
# plt.title('full path software simulation - robot state (x, y) while tracking path')
# plt.xlim(-30,30)
# plt.ylim(-30,30)
# plt.plot(softsim['pf_state_x'], softsim['pf_state_y'], label='software simulation')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# plt.legend()


# plt.figure()
# plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0.25,0.25) -> (0,0,0)')
# plt.subplot(4,1,1)
# plt.xlim(-30,30)
# plt.ylim(-30,30)
# plt.plot(video_pos_cm[0:stage1end]['x'], video_pos_cm[0:stage1end]['y'], label='video odometry')
# plt.plot(odometry['pf_state_x'][0:159],odometry['pf_state_y'][0:159], label='hardware-in-the-loop simulation')
# plt.plot(softsim['pf_state_x'][0:118], softsim['pf_state_y'][0:118], label='software simulation')

# plt.plot(video_pos_cm['x'][0], video_pos_cm['y'][0], 'go')
# plt.plot(video_pos_cm['x'][stage1end], video_pos_cm['y'][stage1end], 'rx')
# plt.ylabel('y pos (cm)')
# plt.legend()
# plt.subplot(4,1,2)
# plt.ylim(-math.pi,math.pi)
# plt.plot(video_heading[0:stage1end], label='video odometry')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,3)
# plt.ylim(-math.pi,math.pi)
# plt.plot(odometry['pf_state_theta'][0:159], label='hardware-in-the-loop simulation')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,4)
# plt.ylim(-math.pi,math.pi)
# plt.plot(softsim['pf_state_theta'][0:118], label='software simulation')
# plt.ylabel('heading (rad)')
# plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
# plt.legend()

# plt.figure()
# plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0,0.25,0) -> (0,0,0)')
# plt.subplot(4,1,1)
# plt.xlim(-30,30)
# plt.ylim(-30,30)
# plt.plot(video_pos_cm[stage1end:stage2end]['x'], video_pos_cm[stage1end:stage2end]['y'], label='video odometry')
# plt.plot(odometry['pf_state_x'][159:309],odometry['pf_state_y'][159:309], label='hardware-in-the-loop simulation')
# plt.plot(softsim['pf_state_x'][118:250], softsim['pf_state_y'][118:250], label='software simulation')

# plt.plot(video_pos_cm['x'][stage1end], video_pos_cm['y'][stage1end], 'go')
# plt.plot(video_pos_cm['x'][stage2end], video_pos_cm['y'][stage2end], 'rx')
# plt.ylabel('y pos (cm)')
# plt.legend()
# plt.subplot(4,1,2)
# plt.plot(video_heading[stage1end:stage2end], label='video odometry')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,3)
# plt.plot(odometry['pf_state_theta'][159:309], label='hardware-in-the-loop simulation')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,4)
# plt.ylim(-math.pi,math.pi)
# plt.plot(softsim['pf_state_theta'][118:250], label='software simulation')
# plt.ylabel('heading (rad)')
# plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
# plt.legend()

# plt.figure()
# plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0,0,2.7) -> (0,0,-2.7) -> (0,0,2.7) -> (0,0,0)')
# plt.subplot(4,1,1)
# plt.xlim(-30,30)
# plt.ylim(-30,30)
# plt.plot(video_pos_cm[stage2end:stage3end]['x'], video_pos_cm[stage2end:stage3end]['y'], label='video odometry')
# plt.plot(odometry['pf_state_x'][309:374],odometry['pf_state_y'][309:374], label='hardware-in-the-loop simulation')
# plt.plot(softsim['pf_state_x'][250:344], softsim['pf_state_y'][250:344], label='software simulation')
# plt.plot(video_pos_cm['x'][stage2end], video_pos_cm['y'][stage2end], 'go')
# plt.plot(video_pos_cm['x'][stage3end], video_pos_cm['y'][stage3end], 'rx')
# plt.ylabel('y pos (cm)')
# plt.legend()
# plt.subplot(4,1,2)
# plt.plot(video_heading[stage2end:stage3end], label='video odometry')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,3)
# plt.plot(odometry['pf_state_theta'][309:374], label='hardware-in-the-loop simulation')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,4)
# plt.ylim(-math.pi,math.pi)
# plt.plot(softsim['pf_state_theta'][250:344], label='software simulation')
# plt.ylabel('heading (rad)')
# plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
# plt.legend()

# fig = plt.figure()
# plt.suptitle('robot state (x, y, theta) while tracking path \n (0,0,0) -> (0.25,0,pi) -> (0,0,pi)')
# plt.subplot(4,1,1)
# plt.xlim(-30,30)
# plt.ylim(-30,30)
# plt.plot(video_pos_cm[stage3end:stage4end]['x'], video_pos_cm[stage3end:stage4end]['y'], label='video odometry')
# plt.plot(odometry['pf_state_x'][374:500],odometry['pf_state_y'][374:500], label='hardware-in-the-loop simulation')
# plt.plot(softsim['pf_state_x'][344:460], softsim['pf_state_y'][344:460], label='software simulation')
# plt.plot(video_pos_cm['x'][stage3end], video_pos_cm['y'][stage3end], 'go')
# plt.plot(video_pos_cm['x'][stage4end], video_pos_cm['y'][stage4end], 'rx')
# plt.ylabel('y pos (cm)')
# plt.legend()
# plt.subplot(4,1,2)
# plt.plot(video_heading[stage3end:stage4end], label='video odometry')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,3)
# plt.plot(odometry['pf_state_theta'][374:500], label='hardware-in-the-loop simulation')
# plt.ylabel('heading (rad)')
# plt.legend()
# plt.subplot(4,1,4)
# plt.ylim(-math.pi,math.pi)
# plt.plot(softsim['state_est_theta'][344:560], label='software simulation')
# plt.ylabel('heading (rad)')
# plt.xlabel('1. x pos (cm), 2. frame number, 3. host computer sensor reading number \n 4. host comp. reading number')
# plt.legend()



plt.show()

