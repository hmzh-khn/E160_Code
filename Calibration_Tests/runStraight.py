"""
Learn alpha using the encoder readings assuming a proportional control model. 

TODO: Make it less dependent on correct path.
"""
import sys
sys.path.insert(0, "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/")

from E160_config import CONFIG_DELTA_T, CONFIG_RAMP_CONSTANT
from E160_environment import *
from E160_graphics import *
from math import sqrt
from random import gauss
from statistics import mean, stdev
import time

IS_DEBUG = True

DESIRED_POWER_PERCENT = 20
DESIRED_POWER = DESIRED_POWER_PERCENT *(256/100)  # percentage of total power on left (slower) motor
ALPHA = 0.9411
NUM_TICKS_TO_RUN = 30000 # ~4.5 meters

def runRobot(env, graphics=None, deltaT=CONFIG_DELTA_T):
  if graphics:
    # update graphics, but stop the thread if user stopped the gui
    if not graphics.update():
        return 1
  
  # update robots
  env.update_robots(deltaT)
  
  # log all the robot data
  env.log_data()

  # maintain timing
  time.sleep(deltaT)

  return 0


if __name__ == "__main__":
  env = E160_environment()
  graphics = E160_graphics(env)

  robot = env.robots[0]
  robot.testing_power_L = DESIRED_POWER
  robot.R_motor_scaling_factor = ALPHA
  robot.file_name = "Log/RunStraight_LeftWheel_P"+str(DESIRED_POWER_PERCENT)+"_Alpha"+str(ALPHA)+"_Ticks_"+str(NUM_TICKS_TO_RUN)+'_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S')+".txt"
  robot.change_headers()

  num_trials = 0

  # number of ticks ran
  num_ticks = 0
  totalTheta = 0

  while abs(num_ticks) < NUM_TICKS_TO_RUN:

    # measure the error between the encoder readings
    deltaS, deltaTheta = robot.delta_state
    print('dTheta - ', deltaTheta) 
    print('num ticks - ', num_ticks)    

    # increment number of trials, number of ticks (on left wheel)
    num_trials = num_trials + 1
    num_ticks += robot.delta_left
    totalTheta += deltaTheta

    if runRobot(env, graphics=graphics):
       break

  robot.testing_power_L = 0
  runRobot(env, graphics=graphics)
  print("Ran for ", num_ticks, "ticks, and deviated by theta =", totalTheta,".")




