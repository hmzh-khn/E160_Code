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

RESET_STATE = (0,0,0)

# 1. Go straight forward (0,0,0) --> (-1,0,0)
TEST1 = [(-0.25,0,0), RESET_STATE]

# 2. Go straight backward (0,0,0) --> (1,0,0)
TEST2 = [(0.25,0,0), RESET_STATE]

# 3a. Rotate (0,0,0) --> (0,0,1rad)
TEST3a = [(0,0,0.25), RESET_STATE]

# 3b. Rotate along shortest direction (0,0,0) --> (0,0,pi) --> (0,0,-pi)
TEST3b = [(0,0,3), (0,0,-3), RESET_STATE]

# 3c. Rotate (0,0,0) --> (0,0,pi/2)
TEST3c = [(0,0,1.57), RESET_STATE]

# 3d. Rotate (0,0,0) --> (0,0,-pi/4)
TEST3d = [(0,0,1.57/2), RESET_STATE]

# 4a. Perpendicular (0,0,0) -> (0,1,0)
TEST4a = [(0,0.25,0), RESET_STATE]

# 4b. Perpendicular (0,0,0) -> (0,-1,0)
TEST4b = [(0,-0.25,0), RESET_STATE]

# 5. Diagonal (0,0,0) -> (1,1,0)
TEST5 = [(0.25,0.25,0), RESET_STATE]

# 6. Close Spot (0,0,0) -> (-0.1,-0.1,0)
TEST6 = [(-0.1,-0.1,0), RESET_STATE]

MAIN_TESTS = (
              TEST1 + 
              TEST2 + 
              TEST3a + 
              TEST3b + 
              TEST3c + 
              TEST3d + 
              TEST4a +
              TEST4b +
              TEST5 + 
              TEST6)


# TEST_PATH = [(1,0,0), (-1,0,0)]


DESIRED_COORDS = MAIN_TESTS
# CLARK_TESTS = [(0.25,0.25,0), RESET_STATE, (0,0.25,0), RESET_STATE, (0,0,2.7), (0,0,-2.7), (0,0,2.7), RESET_STATE, (-0.25,0,3.14), (0,0,3.14)]
# DESIRED_COORDS = CLARK_TESTS

PATH = [E160_state(x=point) for point in DESIRED_COORDS ]

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
  robot.path_tracking_pause_duration = 1 # sec
  # path_tracking = robot.create_path_tracker(PATH)

  # robot.point_tracked = False
  # robot.state_des = E160_state((1,0,0))

  while True:
    runRobot(env, graphics=graphics)

