"""
Learn alpha using the encoder readings. 

TODO: Make it less dependent on correct path.
"""
import sys
sys.path.insert(0, "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/")

from E160_config import CONFIG_DELTA_T
from E160_environment import *
from E160_graphics import *
from math import sqrt
from random import gauss
from statistics import mean, stdev
import time

IS_DEBUG = True

DESIRED_POWER = 30 *(256/100)  # percentage of total power on left (slower) motor
INIT_ALPHA = 1           # assumes identical motors
STOP_THRESHOLD = 0.00001   # stops when change in alpha is this or smaller
MAX_TRIALS = 10000       # maximum number of trials before stops
WINDOW_WIDTH = 10        # number of points across which to calculate mean, stdev
INIT_LEARNING_RATE = 1.0 # how much impact should a single reading have on the next alpha

def initialize_alpha_list(init_alpha=INIT_ALPHA, 
                          max_trials=MAX_TRIALS, 
                          window_width=WINDOW_WIDTH):
  alphas = []

  # make the list initialization-proof by adding values around the initial alpha
  for i in range(WINDOW_WIDTH):
    # draw from normal distribution with mean of INIT_ALPHA,
    #                                    stdev of 2nd root of STOP_THRESHOLD
    init_number = gauss(INIT_ALPHA, sqrt(STOP_THRESHOLD))
    alphas.append(init_number)
    print(i)
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


if __name__ == "__main__":
  env = E160_environment()
  graphics = E160_graphics(env)

  alphas = initialize_alpha_list()
  learning_rate = INIT_LEARNING_RATE

  robot = env.robots[0]
  robot.testing_power_L = DESIRED_POWER

  num_trials = 0

  mean_alpha = mean(alphas[0:WINDOW_WIDTH])
  std_alpha = stdev(alphas[0:WINDOW_WIDTH])

  while std_alpha >= STOP_THRESHOLD:

    k = num_trials + WINDOW_WIDTH

    # measure the error between the encoder readings
    deltaS, deltaTheta = robot.delta_state
    print('dTheta - ', deltaTheta)    

    alphas[k] = learning_rate*(deltaTheta) + alphas[k-1]

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
    learning_rate = min(1, 10/(num_trials+1))

    if runRobot(env, graphics=graphics):
       break

  robot.testing_power_L = 0
  print("After ", num_trials, " updates, the alpha value is ", alphas[k])




