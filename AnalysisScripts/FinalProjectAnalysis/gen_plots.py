"""
Given data from the final project tests, plot
  1. error, 
  2. covariance diagonal terms,
  3. 
"""
import matplotlib
# print(matplotlib.get_backend())
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import linregress
import math

CONVERT_M_TO_CM = 100.0

PATH_HEAD = "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/Log/"

# CHANGE TO FILE NAMES OF ACTUAL TESTS
FILE = PATH_HEAD + "SMALL_COURSE_HARDWARE MODE_3Sensors_18-04-28 21.56.52.txt"
data = pd.read_csv(FILE, sep=" ")

# MAYBE NEED SOME DIRECTION FLIPPING FOR SIMULATION FILES
data['state_odo_x'] = CONVERT_M_TO_CM * data['state_odo_x']
data['state_odo_y'] = CONVERT_M_TO_CM * data['state_odo_y']
data['state_odo_theta'] = data['state_odo_theta']
data['pf_state_x'] = CONVERT_M_TO_CM * data['pf_state_x']
data['pf_state_y'] = CONVERT_M_TO_CM * data['pf_state_y']
data['pf_state_theta'] = data['pf_state_theta']

dist_error = np.sqrt(data['est_error_x']**2 )
dist_error = np.sqrt(data['est_error_y']**2)
distance_rms = np.sqrt(np.mean(dist_error**2))
print('Translation RMS - ', distance_rms)

def normalize_angle(theta):
  '''
  Returns an angle in range (-pi, pi]
  '''
  out_angle = theta % (2 * math.pi)
  greater_than_pi = (out_angle > math.pi).astype(int)
  return out_angle - 2 * math.pi * greater_than_pi


# example of printing path - translation
plt.figure()
plt.title('robot translational path odometry v. ukf')
plt.plot(data['state_odo_x'], data['state_odo_y'], 'bo', label='odometry position')
plt.plot(data['pf_state_x'], data['pf_state_y'], 'rx', label='ukf position')
plt.xlabel('x pos (cm)')
plt.ylabel('y pos (cm)')
# plt.xlim(-30,150)
# plt.ylim(-100,45)
plt.legend()


# example of printing path - translation
plt.figure()
plt.title('robot angular path odometry v. ukf')
plt.plot(data['state_odo_theta'], 'bo', label='odometry angle')
plt.plot(data['pf_state_theta'], 'rx', label='ukf angle')
plt.xlabel('sample number')
plt.ylabel('angle (rad)')
# plt.xlim(-30,150)
plt.ylim(-np.pi, np.pi)
plt.legend()


# example of printing position error
plt.figure()
plt.title('robot translational path distance error odometry v. ukf')
plt.plot(data['est_error_x'], 'bo', label='error in x')
plt.plot(data['est_error_y'], 'rx', label='error in y')
plt.xlabel('sample number')
plt.ylabel('distance error (m)')
# plt.xlim(-30,150)
# plt.ylim(-np.pi, np.pi)
plt.legend()


# example of printing angular error
plt.figure()
plt.title('robot angular path error odometry v. ukf')
plt.plot(data['est_error_theta'], 'bo', label='error in theta')
plt.xlabel('sample number')
plt.ylabel('angle error (rad)')
# plt.xlim(-30,150)
# plt.ylim(-np.pi, np.pi)
plt.legend()


# example of printing translational variance
plt.figure()
plt.title('robot translational path variance ukf')
plt.plot(data['variance00'], 'bo', label='variance in x w.r.t. x')
plt.plot(data['variance11'], 'rx', label='variance in y w.r.t. y')
plt.xlabel('sample number')
plt.ylabel('distance variance (m^2)')
# plt.xlim(-30,150)
# plt.ylim(-np.pi, np.pi)
plt.legend()

plt.figure()
plt.title('robot angular path variance ukf')
plt.plot(data['variance22'], 'bo', label='variance in theta w.r.t. theta')
# plt.plot(data['variance11'], 'rx', label='variance in y w.r.t. y')
plt.xlabel('sample number')
plt.ylabel('angular variance (rad^2)')
# plt.xlim(-30,150)
# plt.ylim(-np.pi, np.pi)
plt.legend()


# plt.figure()
# plt.title('full path state (x,y) - known start in simulation (400 particles)')
# plt.plot(softsim['state_odo_x'], softsim['state_odo_y'], 'bo', label='knownstart_sim_odo')
# plt.plot(softsim['pf_state_x'], softsim['pf_state_y'], 'rx', label='knownstart_sim_pf')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# plt.xlim(-30,150)
# plt.ylim(-100,45)
# plt.legend()


# # path locations
# ODOMETRY_FILEPATH   = PATH_HEAD+"KnownStart_P400_Hardware.csv"
# SIMULATION_FILEPATH = PATH_HEAD+"KnownStart_P400_Simulation.csv"

# CONVERT_M_TO_CM = 100

# # get data from files
# hardsim = pd.read_csv(ODOMETRY_FILEPATH, sep=" ")
# softsim = pd.read_csv(SIMULATION_FILEPATH, sep=" ")

# flip reversed axes in odometry, software simulation
# hardsim['pf_state_x'] = CONVERT_M_TO_CM * hardsim['pf_state_x']
# hardsim['pf_state_y'] = CONVERT_M_TO_CM * hardsim['pf_state_y']
# hardsim['pf_state_theta'] = hardsim['pf_state_theta']

# softsim['pf_state_x'] = CONVERT_M_TO_CM * softsim['pf_state_x']
# softsim['pf_state_y'] = CONVERT_M_TO_CM * softsim['pf_state_y']
# softsim['pf_state_theta'] = softsim['pf_state_theta']

# hardsim['state_odo_x'] = CONVERT_M_TO_CM * hardsim['state_odo_x']
# hardsim['state_odo_y'] = CONVERT_M_TO_CM * hardsim['state_odo_y']
# hardsim['state_odo_theta'] = hardsim['state_odo_theta']

# softsim['state_odo_x'] = CONVERT_M_TO_CM * softsim['state_odo_x']
# softsim['state_odo_y'] = CONVERT_M_TO_CM * softsim['state_odo_y']
# softsim['state_odo_theta'] = softsim['state_odo_theta']

# plt.figure()
# plt.title('full path state (x,y) - known start in simulation (400 particles)')
# plt.plot(softsim['state_odo_x'], softsim['state_odo_y'], 'bo', label='knownstart_sim_odo')
# plt.plot(softsim['pf_state_x'], softsim['pf_state_y'], 'rx', label='knownstart_sim_pf')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# plt.xlim(-30,150)
# plt.ylim(-100,45)
# plt.legend()

# plt.figure()
# plt.title('full path state (x,y) - known start in hardware (400 particles)')
# plt.plot(hardsim['state_odo_x'], hardsim['state_odo_y'], 'bo', label='knownstart_hardware_odo')
# plt.plot(hardsim['pf_state_x'], hardsim['pf_state_y'], 'rx', label='knownstart_hardware_pf')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# plt.xlim(-30,150)
# plt.ylim(-100,45)
# plt.legend()

# plt.figure()
# plt.title('full path state (theta) - known start in simulation (400 particles)')
# plt.plot(softsim['state_odo_theta'], 'bo', label='knownstart_sim_odo_theta')
# plt.plot(softsim['pf_state_theta'], 'rx', label='knownstart_sim_pf_theta')
# plt.xlabel('num samples')
# plt.ylabel('global heading (rad)')
# plt.ylim(-3.14,3.14)
# # plt.ylim(-100,45)
# plt.legend()

# plt.figure()
# plt.title('full path state (theta) - known start in hardware (400 particles)')
# plt.plot(hardsim['state_odo_theta'], 'bo', label='knownstart_hardware_odo_theta')
# plt.plot(hardsim['pf_state_theta'], 'rx', label='knownstart_hardware_pf_theta')
# plt.xlabel('num samples')
# plt.ylabel('global heading (rad)')
# plt.ylim(-3.14,3.14)
# # plt.ylim(-100,45)
# plt.legend()





# path locations
# ODOMETRY_FILEPATH   = PATH_HEAD+"RandomStart_P400_Hardware.csv"
# SIMULATION_FILEPATH = PATH_HEAD+"RandomStart_P400_Simulation.csv"

# CONVERT_M_TO_CM = 100

# # get data from files
# hardsim = pd.read_csv(ODOMETRY_FILEPATH, sep=" ")
# softsim = pd.read_csv(SIMULATION_FILEPATH, sep=" ")

# # flip reversed axes in odometry, software simulation
# hardsim['pf_state_x'] = CONVERT_M_TO_CM * hardsim['pf_state_x']
# hardsim['pf_state_y'] = CONVERT_M_TO_CM * hardsim['pf_state_y']
# hardsim['pf_state_theta'] = hardsim['pf_state_theta']

# softsim['pf_state_x'] = CONVERT_M_TO_CM * softsim['pf_state_x']
# softsim['pf_state_y'] = CONVERT_M_TO_CM * softsim['pf_state_y']
# softsim['pf_state_theta'] = softsim['pf_state_theta']

# hardsim['state_odo_x'] = CONVERT_M_TO_CM * hardsim['state_odo_x']
# hardsim['state_odo_y'] = CONVERT_M_TO_CM * hardsim['state_odo_y']
# hardsim['state_odo_theta'] = hardsim['state_odo_theta']

# softsim['state_odo_x'] = CONVERT_M_TO_CM * softsim['state_odo_x']
# softsim['state_odo_y'] = CONVERT_M_TO_CM * softsim['state_odo_y']
# softsim['state_odo_theta'] = softsim['state_odo_theta']

# # plt.figure()
# # plt.title('full path state (x,y) - random start in simulation (400 particles)')
# # plt.plot(softsim['state_odo_x'], softsim['state_odo_y'], 'bo', label='randomstart_sim_odo')
# # plt.plot(softsim['pf_state_x'], softsim['pf_state_y'], 'rx', label='randomstart_sim_pf')
# # plt.xlabel('x pos (cm)')
# # plt.ylabel('y pos (cm)')
# # plt.xlim(-30,150)
# # plt.ylim(-100,45)
# # plt.legend()

# # plt.figure()
# # plt.title('full path state (x,y) - random start in hardware (400 particles)')
# # plt.plot(hardsim['state_odo_x'], hardsim['state_odo_y'], 'bo', label='randomstart_hardware_odo')
# # plt.plot(hardsim['pf_state_x'], hardsim['pf_state_y'], 'rx', label='randomstart_hardware_pf')
# # plt.xlabel('x pos (cm)')
# # plt.ylabel('y pos (cm)')
# # plt.xlim(-30,150)
# # plt.ylim(-100,45)
# # plt.legend()

# plt.figure()
# plt.title('full path state (theta) - random start in simulation (400 particles)')
# plt.plot(softsim['state_odo_theta'], 'bo', label='randomstart_sim_odo_theta')
# plt.plot(softsim['pf_state_theta'], 'rx', label='randomstart_sim_pf_theta')
# plt.xlabel('num samples')
# plt.ylabel('global heading (rad)')
# plt.ylim(-3.14,3.14)
# # plt.ylim(-100,45)
# plt.legend()

# plt.figure()
# plt.title('full path state (theta) - random start in hardware (400 particles)')
# plt.plot(hardsim['state_odo_theta'], 'bo', label='randomstart_hardware_odo_theta')
# plt.plot(hardsim['pf_state_theta'], 'rx', label='randomstart_hardware_pf_theta')
# plt.xlabel('num samples')
# plt.ylabel('global heading (rad)')
# plt.ylim(-3.14,3.14)
# # plt.ylim(-100,45)
# plt.legend()


# path locations
# ODOMETRY_FILEPATH   = PATH_HEAD+"KnownStart_PFControl_P400_Hardware.csv"
# SIMULATION_FILEPATH = PATH_HEAD+"KnownStart_PFControl_P400_Simulation.csv"

# CONVERT_M_TO_CM = 100

# # get data from files
# hardsim = pd.read_csv(ODOMETRY_FILEPATH, sep=" ")
# softsim = pd.read_csv(SIMULATION_FILEPATH, sep=" ")

# # flip reversed axes in odometry, software simulation
# hardsim['pf_state_x'] = CONVERT_M_TO_CM * hardsim['pf_state_x']
# hardsim['pf_state_y'] = CONVERT_M_TO_CM * hardsim['pf_state_y']
# hardsim['pf_state_theta'] = hardsim['pf_state_theta']

# softsim['pf_state_x'] = CONVERT_M_TO_CM * softsim['pf_state_x']
# softsim['pf_state_y'] = CONVERT_M_TO_CM * softsim['pf_state_y']
# softsim['pf_state_theta'] = softsim['pf_state_theta']

# hardsim['state_odo_x'] = CONVERT_M_TO_CM * hardsim['state_odo_x']
# hardsim['state_odo_y'] = CONVERT_M_TO_CM * hardsim['state_odo_y']
# hardsim['state_odo_theta'] = hardsim['state_odo_theta']

# softsim['state_odo_x'] = CONVERT_M_TO_CM * softsim['state_odo_x']
# softsim['state_odo_y'] = CONVERT_M_TO_CM * softsim['state_odo_y']
# softsim['state_odo_theta'] = softsim['state_odo_theta']

# plt.figure()
# plt.title('full path state (x,y) - known start, PF control in simulation (400 particles)')
# plt.plot(softsim['state_odo_x'], softsim['state_odo_y'], 'bo', label='knownstart_sim_odo')
# plt.plot(softsim['pf_state_x'], softsim['pf_state_y'], 'rx', label='knownstart_sim_pf')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# plt.xlim(-30,150)
# plt.ylim(-100,45)
# plt.legend()

# plt.figure()
# plt.title('full path state (x,y) - known start, PF control in hardware (400 particles)')
# plt.plot(hardsim['state_odo_x'], hardsim['state_odo_y'], 'bo', label='knownstart_hardware_odo')
# plt.plot(hardsim['pf_state_x'], hardsim['pf_state_y'], 'rx', label='knownstart_hardware_pf')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# plt.xlim(-30,150)
# plt.ylim(-100,45)
# plt.legend()

# plt.figure()
# plt.title('full path state (theta) - known start, PF control in simulation (400 particles)')
# plt.plot(softsim['state_odo_theta'], 'bo', label='knownstart_sim_odo_theta')
# plt.plot(softsim['pf_state_theta'], 'rx', label='knownstart_sim_pf_theta')
# plt.xlabel('num samples')
# plt.ylabel('global heading (rad)')
# plt.ylim(-3.14,3.14)
# # plt.ylim(-100,45)
# plt.legend()

# plt.figure()
# plt.title('full path state (theta) - known start, PF control in hardware (400 particles)')
# plt.plot(hardsim['state_odo_theta'], 'bo', label='knownstart_hardware_odo_theta')
# plt.plot(hardsim['pf_state_theta'], 'rx', label='knownstart_hardware_pf_theta')
# plt.xlabel('num samples')
# plt.ylabel('global heading (rad)')
# plt.ylim(-3.14,3.14)
# # plt.ylim(-100,45)
# plt.legend()


plt.show()

