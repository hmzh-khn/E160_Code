"""
E160_config.py
A central location for all configuration variables.
Author: Hamzah Khan
Date Created: Jan 27, 2018
Date Last Edited: Jan 27, 2018
"""
import math


""" 
Set config for E160_environment.py
"""
# Computer XBee port info
CONFIG_PORT = "COM3"

HARDWARE_MODE = "HARDWARE MODE"
SIMULATION_MODE = "SIMULATION MODE"

MANUAL_CONTROL_MODE = "MANUAL CONTROL MODE"
AUTONOMOUS_CONTROL_MODE = "AUTONOMOUS CONTROL MODE"



""" 
Set config for E160_graphics.py
"""
CONFIG_WINDOW_SCALE = 250



""" 
Set config for E160_robot.py
"""
# accounts for right motor moving faster than left motor
CONFIG_R_MOD_FRACTION = 0.99


CONFIG_ERROR_THRESHOLD_CM = 0.8


CONFIG_MAX_POWER = 25
CONFIG_MIN_POWER = 10


#### SET FORWARD DISTANCE SENSOR CALIBRATION #####
def CONFIG_FORWARD_DISTANCE_CALIBRATION(potential_measurement):
  return -31.546 * math.log(math.exp(-7.19)*potential_measurement)
  # old calibration with 10cm -32.05 * math.log(math.exp(-7.07)*range_measurements[0])


"""
Lab 1 configurations.
"""
CONFIG_ERROR_DISTANCE_CM = 1000000
CONFIG_PROPORTIONAL_CONSTANT = 100


""" 
Set all of the regularly altered configurations here.
"""

##### SET DESIRED STOP DISTANCE MODE HERE (for Lab 1) #####
CONFIG_OFFSET_CM = 0.5*2.54
CONFIG_DESIRED_DISTANCE_CM = 30.0 - CONFIG_OFFSET_CM

##### SET HARDWARE MODE HERE (Simulation or Hardware) #####
CONFIG_ROBOT_MODE = HARDWARE_MODE
##### SET CONTROL MODE HERE (Manual or Autonomous) #####
CONFIG_CONTROL_MODE = AUTONOMOUS_CONTROL_MODE


