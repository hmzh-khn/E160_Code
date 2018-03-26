"""
E160_config.py
A central location for all configuration variables.
Author: Hamzah Khan
Date Created: Jan 27, 2018
Date Last Edited: Jan 27, 2018
"""
import math
import platform
import sys

""" 
Set config for E160_environment.py
"""
print("Running on ", platform.system(), ", Python v", sys.version_info[0])

# Computer XBee port info
if platform.system() == "Windows":
  CONFIG_PORT = "COM3"
else:
  CONFIG_PORT = "/dev/tty.usbserial-DN02Z6QO"


HARDWARE_MODE = "HARDWARE MODE"
SIMULATION_MODE = "SIMULATION MODE"

MANUAL_CONTROL_MODE = "MANUAL CONTROL MODE"
AUTONOMOUS_CONTROL_MODE = "AUTONOMOUS CONTROL MODE"

INDOOR_COURSE = "INDOOR"
INDOOR_MIN_X_M = -0.5
INDOOR_MAX_X_M = 1.5
INDOOR_MIN_Y_M = -1.4
INDOOR_MAX_Y_M = 0.525

# for PI controller, either immediate or slow stop
# ERROR? WHEN TICK RATE IS 0 IN TRANSITION, MAY HAVE ERRORS!
CONFIG_IMMEDIATE_STOP_TICK_RATE = 0
CONFIG_SLOW_STOP_TICK_RATE = 1

CONFIG_DELTA_T = 0.05 # seconds



""" 
Set config for E160_graphics.py
"""
CONFIG_WINDOW_SCALE = 100


##### the intervals for setting the manual velocities
CONFIG_SCALE_RESOLUTION = 5



""" 
Set config for E160_robot.py
"""
# accounts for right motor moving faster than left motor
CONFIG_R_MOTOR_SCALING_FACTOR = 1

CONFIG_RAMP_PERCENT_CONSTANT = 20
CONFIG_RAMP_CONSTANT = 2.56*CONFIG_RAMP_PERCENT_CONSTANT # scale to 256 bit power

CONFIG_ERROR_THRESHOLD_CM = 0.8


CONFIG_MAX_POWER = 25
CONFIG_MIN_POWER = 10


#### SET FORWARD DISTANCE SENSOR CALIBRATION #####
def CONFIG_FORWARD_DISTANCE_CALIBRATION(potential_measurement):
  return -31.546 * math.log(math.exp(-7.19)*potential_measurement)
  # old calibration with 10cm -32.05 * math.log(math.exp(-7.07)*range_measurements[0])

# def CONFIG_FORWARD_DISTANCE_CALIBRATION(potential_measurement):
  # return -31.546 * math.log(math.exp(-7.19)*potential_measurement)

"""
Lab 1 configurations.
"""
CONFIG_ERROR_DISTANCE_CM = 1000000
CONFIG_PROPORTIONAL_CONSTANT = 100


"""
Lab 3 configurations.
"""
CONFIG_DISTANCE_THRESHOLD_M = 0.03
CONFIG_ANGLE_THRESHOLD_RAD = 0.05

def CONFIG_IN_HARDWARE_MODE(robot_mode):
  return robot_mode == HARDWARE_MODE

def CONFIG_IN_SIMULATION_MODE(robot_mode):
  return robot_mode == SIMULATION_MODE

# decides the threshold away from a quarter turn that the point tracker switches direction
CONFIG_POINT_TRACKING_ANGLE_BIAS = math.pi/4


"""
Config for lab 4
"""
CONFIG_HEADING_TOLERANCE = 0.05

"""
Tick maps (cm per tick)
"""
# takes 
CONFIG_LEFT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP = {
  10: 0.015375,
  40: 0.01546,
  70: 0.015515,
  100: 0.015774603153597,
}

CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP = {
  10: 0.015375,
  40: 0.01546,
  70: 0.015515,
  100: 0.015774603153597,
}


"""
Conversions
"""
CONFIG_CM_TO_M = 0.01
CONFIG_M_TO_CM = 100
CONFIG_IN_TO_CM = 2.54
CONFIG_IN_TO_M = CONFIG_IN_TO_CM * CONFIG_CM_TO_M
CONFIG_IN_TO_M = 0.0254
CONFIG_M_TO_IN = 39.37
CONFIG_DEGS_PER_REVOLUTION = 360 # degrees per revolution (i.e. per 2 pi)


"""
Known Angles
"""
CONFIG_EIGHTH_TURN = math.pi/4
CONFIG_QUARTER_TURN = math.pi/2
CONFIG_HALF_TURN = math.pi
CONFIG_FULL_TURN = 2*math.pi


""" 
Set all of the regularly altered configurations here.
"""

CONFIG_LAB_NUMBER = 4

##### SET DESIRED STOP DISTANCE MODE HERE (for Lab 1) #####
CONFIG_OFFSET_CM = 0.5*2.54
CONFIG_DESIRED_DISTANCE_CM = 30.0 - CONFIG_OFFSET_CM

##### SET OFFSET OF SENSORS FROM SENSOR #####
CONFIG_FRONT_SENSOR_OFFSET_M = 2.75 * CONFIG_IN_TO_M
CONFIG_RIGHT_SENSOR_OFFSET_M = 4 * CONFIG_IN_TO_M
CONFIG_LEFT_SENSOR_OFFSET_M = 3.75 * CONFIG_IN_TO_M

##### SET MAP #####
CONFIG_COURSE = INDOOR_COURSE
CONFIG_MAP_MIN_X_M = INDOOR_MIN_X_M
CONFIG_MAP_MAX_X_M = INDOOR_MAX_X_M
CONFIG_MAP_MIN_Y_M = INDOOR_MIN_Y_M
CONFIG_MAP_MAX_Y_M = INDOOR_MAX_Y_M

##### SET HARDWARE MODE HERE (Simulation or Hardware) #####
CONFIG_ROBOT_MODE = HARDWARE_MODE

##### SET CONTROL MODE HERE (Manual or Autonomous) #####
CONFIG_CONTROL_MODE = MANUAL_CONTROL_MODE

