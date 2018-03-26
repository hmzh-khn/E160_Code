"""
Learn alpha using the encoder readings assuming a proportional control model. 

TODO: Make it less dependent on correct path.
"""
import sys
sys.path.insert(0, "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/")

# from collections import defaultdict
from E160_config import CONFIG_DELTA_T, CONFIG_RAMP_CONSTANT
from E160_environment import *
from E160_graphics import *
from math import sqrt
from random import gauss
from statistics import mean, stdev
import time

IS_DEBUG = True

DESIRED_POWER_PERCENT = 80
DESIRED_POWER = DESIRED_POWER_PERCENT *(256/100)  # percentage of total power on left (slower) motor
INIT_ALPHA = 1           # assumes identical motors
STOP_THRESHOLD = 0.0001 # stops when change in alpha is this or smaller
MAX_TRIALS = 10000       # maximum number of trials before stops
WINDOW_WIDTH = 10        # number of points across which to calculate mean, stdev
INIT_LEARNING_RATE = 1.0 # how much impact should a single reading have on the next alpha

learning_coefficient = 1

def initialize_alpha_list(init_alpha=INIT_ALPHA,
                          init_stdev=sqrt(STOP_THRESHOLD),
                          max_trials=MAX_TRIALS, 
                          window_width=WINDOW_WIDTH):
  alphas = []

  # make the list initialization-proof by adding values around the initial alpha
  for _ in range(WINDOW_WIDTH):
    # draw from normal distribution with mean of INIT_ALPHA,
    #                                    stdev of 2nd root of STOP_THRESHOLD
    init_number = gauss(init_alpha, init_stdev)
    alphas.append(init_number)
  alphas = alphas + (MAX_TRIALS - WINDOW_WIDTH)*[0]

  return alphas

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

# dictionary of known alpha values
# ALPHAS_MAP = {0: 1,
#               20: 0.94115,
#               40: 0.97853}


if __name__ == "__main__":
  env = E160_environment()
  graphics = E160_graphics(env)

  alphas = initialize_alpha_list(init_alpha=0.95172, init_stdev=1.5*STOP_THRESHOLD)
  learning_rate = INIT_LEARNING_RATE

  robot = env.robots[0]
  robot.testing_power_L = DESIRED_POWER
  robot.file_name = "Log/P"+str(DESIRED_POWER_PERCENT)+"Forward_LearningCoef"+str(learning_coefficient)+'_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S')+".txt"
  robot.change_headers()

  num_trials = 0

  mean_alpha = mean(alphas[0:WINDOW_WIDTH])
  std_alpha = stdev(alphas[0:WINDOW_WIDTH])

  # num_wait_periods = round((DESIRED_POWER_PERCENT / CONFIG_RAMP_CONSTANT)) + 1
  # for power in range(DESIRED_POWER_PERCENT, CONFIG_RAMP_CONSTANT):
  #   power = min(power, DESIRED_POWER)
  #   robot.R_motor_scaling_factor = ALPHAS_MAP
  #   runRobot(env, graphics=graphics)

  totalTheta = 0
  while std_alpha >= STOP_THRESHOLD:

    k = num_trials + WINDOW_WIDTH

    # measure the error between the encoder readings
    deltaS, deltaTheta = robot.delta_state
    totalTheta += deltaTheta
    print('dTheta - ', deltaTheta) 
    print('totalTheta - ', totalTheta)

    alphas[k] = alphas[k-1] + learning_rate*deltaTheta #+ learning_rate*totalTheta + alphas[k-1]

    # calculate new statistics
    mean_alpha = mean(alphas[k:k-WINDOW_WIDTH:-1])
    std_alpha = stdev(alphas[k:k-WINDOW_WIDTH:-1])

    if IS_DEBUG:
      print("kth alpha stats - ", k)
      print("-------------------")
      print("Next alpha: ", alphas[k])
      print("Mean alpha (last ", WINDOW_WIDTH, ") - ", mean_alpha)
      print("Stdev alpha (last ", WINDOW_WIDTH, ") - ", std_alpha)
      print("\n")

    # set alpha to new value
    robot.R_motor_scaling_factor = alphas[k]
    
    # increment number of trials
    num_trials = num_trials + 1

    # update learning rate
    learning_rate = min(1, learning_coefficient/(num_trials+1))

    if runRobot(env, graphics=graphics):
       break

  robot.testing_power_L = 0
  runRobot(env, graphics=graphics)
  print("After ", num_trials, " updates, the alpha value is ", alphas[k])
  print("Deviated by theta =", totalTheta,".")




