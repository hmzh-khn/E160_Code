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

def normalize_angle(theta):
  '''
  Returns an angle in range (-pi, pi]
  '''
  out_angle = theta % (2 * math.pi)
  greater_than_pi = (out_angle > math.pi).astype(int)
  return out_angle - 2 * math.pi * greater_than_pi


CONVERT_M_TO_CM = 100.0

IS_HARD = False
PATH_HEAD = "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/Log/"

files = ["SMALL_COURSE_Noise_Testing_0.05_3_Sensors_18-04-29 10.21.49.txt",
         "SMALL_COURSE_Noise_Testing_0.1_3_Sensors_18-04-29 10.22.40.txt",
         "SMALL_COURSE_Noise_Testing_0.15_3_Sensors_18-04-29 10.23.21.txt",
         "SMALL_COURSE_Noise_Testing_0.2_3_Sensors_18-04-29 10.24.00.txt",
         "SMALL_COURSE_Noise_Testing_0.25_3_Sensors_18-04-29 10.25.00.txt"]

for i in files:
  # CHANGE TO FILE NAMES OF ACTUAL TESTS
  SIM_FILE = PATH_HEAD + i
  # SIM_CTRL_FILE = PATH_HEAD + "SMALL_COURSE_3Sensors_UKFControl_18-04-28 21.31.45.txt"

  HARD_FILE = None
  hard_data = None

  if IS_HARD:
    HARD_FILE = PATH_HEAD + "SMALL_COURSE_HARDWARE MODE_3Sensors_UKF_CONTROL_18-04-28 21.54.45.txt"
    # HARD_CTRL_FILE = PATH_HEAD + "SMALL_COURSE_HARDWARE MODE_3Sensors_UKF_CONTROL_18-04-28 21.54.45.txt"

  sim_data = pd.read_csv(SIM_FILE, sep=" ")
  # sim_ctrl_data = pd.read_csv(SIM_CTRL_FILE sep=" ")
  for i in range(len(sim_data)):
    sim_data['est_error_theta'][i] = normalize_angle(sim_data['est_error_theta'][i])

  if IS_HARD:
    hard_data = pd.read_csv(HARD_FILE, sep=" ")
    for i in range(len(sim_data)):
      hard_data['est_error_theta'][i] = normalize_angle(hard_data['est_error_theta'][i])
  # hard_ctrl_data = pd.read_csv(HARD_CTRL_FILE, sep=" ")


  # MAYBE NEED SOME DIRECTION FLIPPING FOR SIMULATION FILES
  sim_data['state_odo_x'] = CONVERT_M_TO_CM * sim_data['state_odo_x']
  sim_data['state_odo_y'] = CONVERT_M_TO_CM * sim_data['state_odo_y']
  sim_data['variance11'] = -CONVERT_M_TO_CM * sim_data['variance11']
  sim_data['state_odo_theta'] = sim_data['state_odo_theta']
  sim_data['pf_state_x'] = CONVERT_M_TO_CM * sim_data['pf_state_x']
  sim_data['pf_state_y'] = CONVERT_M_TO_CM * sim_data['pf_state_y']
  sim_data['pf_state_theta'] = sim_data['pf_state_theta']

  if IS_HARD:
    hard_data['state_odo_x'] = CONVERT_M_TO_CM * hard_data['state_odo_x']
    hard_data['state_odo_y'] = CONVERT_M_TO_CM * hard_data['state_odo_y']
    hard_data['state_odo_theta'] = hard_data['state_odo_theta']
    hard_data['pf_state_x'] = CONVERT_M_TO_CM * hard_data['pf_state_x']
    hard_data['pf_state_y'] = CONVERT_M_TO_CM * hard_data['pf_state_y']
    hard_data['pf_state_theta'] = hard_data['pf_state_theta']


  dist_sim_error_x = np.sqrt(sim_data['est_error_x']**2)
  dist_sim_error_y = np.sqrt(sim_data['est_error_y']**2)
  dist_sim_error_theta = np.sqrt(sim_data['est_error_theta']**2)
  distance_sim_rms_x = np.sqrt(np.mean(dist_sim_error_x**2))
  distance_sim_rms_y = np.sqrt(np.mean(dist_sim_error_y**2))
  distance_sim_rms_theta = np.sqrt(np.mean(dist_sim_error_theta**2))
  # print('Translation sim RMS x,y - ', distance_sim_rms_x, distance_sim_rms_y)
  print(distance_sim_rms_theta)

  if IS_HARD:
    dist_hard_error_x = np.sqrt(hard_data['est_error_x']**2 )
    dist_hard_error_y = np.sqrt(hard_data['est_error_y']**2)
    distance_hard_rms_x = np.sqrt(np.mean(dist_hard_error_x**2))
    distance_hard_rms_y = np.sqrt(np.mean(dist_hard_error_y**2))
    print('Translation sim RMS x,y - ', distance_hard_rms_x, distance_hard_rms_y)

 
# example of printing path - translation
# plt.figure()
# plt.title('robot translational path odometry v. ukf')
# plt.plot(sim_data['state_odo_x'], sim_data['state_odo_y'], 'bo', label='sim odometry position')
# plt.plot(sim_data['pf_state_x'], sim_data['pf_state_y'], 'rx', label='sim ukf position')
# if IS_HARD:
#   plt.plot(hard_data['state_odo_x'], hard_data['pf_state_y'], 'go', label='hardware odometry position')
#   plt.plot(hard_data['pf_state_x'], hard_data['pf_state_y'], 'cx', label='hardware ukf position')
# plt.xlabel('x pos (cm)')
# plt.ylabel('y pos (cm)')
# # plt.xlim(-30,150)
# # plt.ylim(-100,45)
# plt.legend()


  # example of printing path - translation
  # plt.figure()
  # plt.title('robot angular path odometry v. ukf')
  # plt.plot(sim_data['state_odo_theta'], 'bo', label='sim odometry angle')
  # plt.plot(sim_data['pf_state_theta'], 'rx', label='sim ukf angle')
  # if IS_HARD:
  #   plt.plot(hard_data['state_odo_theta'], 'go', label='hardware odometry angle')
  #   plt.plot(hard_data['pf_state_theta'], 'cx', label='hardware ukf angle')
  # plt.xlabel('sample number')
  # plt.ylabel('angle (rad)')
  # # plt.xlim(-30,150)
  # plt.ylim(-np.pi, np.pi)
  # plt.legend()


# example of printing position error
# plt.figure()
# plt.title('robot translational path distance error odometry v. ukf')
# plt.plot(sim_data['est_error_x'], 'bo', label='sim error in x')
# plt.plot(sim_data['est_error_y'], 'rx', label='sim error in y')
# if IS_HARD:
#   plt.plot(hard_data['est_error_x'], 'go', label='hardware error in x')
#   plt.plot(hard_data['est_error_y'], 'cx', label='hardware error in y')
# plt.xlabel('sample number')
# plt.ylabel('distance error (m)')
# # plt.xlim(-30,150)
# # plt.ylim(-np.pi, np.pi)
# plt.legend()


  # example of printing angular error
  # plt.figure()
  # plt.title('robot angular path error odometry v. ukf')
  # plt.plot(sim_data['est_error_theta'], 'bo', label='sim error in theta')
  # if IS_HARD:
  #   plt.plot(hard_data['est_error_theta'], 'gx', label='hardware error in theta')

  # plt.xlabel('sample number')
  # plt.ylabel('angle error (rad)')
  # # plt.xlim(-30,150)
  # # plt.ylim(-np.pi, np.pi)
  # plt.legend()


  # example of printing translational variance
  # plt.figure()
  # plt.title('robot translational path variance ukf')
  # plt.plot(sim_data['variance00'], 'bo', label='sim variance in x')
  # plt.plot(sim_data['variance11'], 'rx', label='sim variance in y ')
  # if IS_HARD:
  #   plt.plot(hard_data['variance00'], 'go', label='hardware variance in x')
  #   plt.plot(hard_data['variance11'], 'cx', label='hardware variance in y')
  # plt.xlabel('sample number')
  # plt.ylabel('distance variance (m^2)')
  # # plt.xlim(-30,150)
  # # plt.ylim(-np.pi, np.pi)
  # plt.legend()

  # plt.figure()
  # plt.title('robot angular path variance ukf')
  # plt.plot(sim_data['variance22'], 'bo', label='sim variance in theta')
  # if IS_HARD:
  #   plt.plot(hard_data['variance22'], 'go', label='hardware variance in theta')
  # plt.xlabel('sample number')
  # plt.ylabel('angular variance (rad^2)')
  # # plt.xlim(-30,150)
  # # plt.ylim(-np.pi, np.pi)
  # plt.legend()


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

