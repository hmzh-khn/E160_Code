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

PATH_HEAD = "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/Log/"

FILE = PATH_HEAD + "Bot0_18-04-27 21.52.09.txt"
data = pd.read_csv(FILE, sep=" ")
print(data.shape)

def normalize_angle(theta):
  '''
  Returns an angle in range (-pi, pi]
  '''
  out_angle = theta % (2 * math.pi)
  greater_than_pi = (out_angle > math.pi).astype(int)
  return out_angle - 2 * math.pi * greater_than_pi

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


# plt.show()

