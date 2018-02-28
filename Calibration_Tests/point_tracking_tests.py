"""
Use path tracker to automate testing for point tracking.

TODO: Make it less dependent on correct path.
"""
import sys
sys.path.insert(0, "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/")
sys.path.insert(0, "/Users/Loaner/Documents/E160_Code/")

from E160_config import CONFIG_DELTA_T, CONFIG_RAMP_PERCENT_CONSTANT
from E160_environment import *
from E160_graphics import *
from E160_state import *
from math import sqrt
from random import gauss
from statistics import mean, stdev
import time

IS_DEBUG = True

PATH = [(1,0,0), (-1,0,0)]
PATH = [E160_state(x=point) for point in PATH]

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
  robot.path = PATH
  # path_tracking = robot.create_path_tracker(PATH)

  # robot.point_tracked = False
  # robot.state_des = E160_state((1,0,0))

  while True:
    runRobot(env, graphics=graphics)

