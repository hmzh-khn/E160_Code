"""
Calibration script for distance sensing.
"""
import sys
sys.path.insert(0, "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/")

import matplotlib
# print(matplotlib.get_backend())
matplotlib.use("TkAgg")

from E160_config import CONFIG_DELTA_T
from E160_environment import *
from E160_graphics import *
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

NUM_READINGS_PER_TEST = 100
READING_DISTS_CM = np.array([10, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80, 90, 100, 120, 140])
# READING_DISTS_CM = np.array([20, 30, 40])
readings = np.zeros((len(READING_DISTS_CM), NUM_READINGS_PER_TEST))

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
  time.sleep(0.1*deltaT)

  return 0

if __name__ == '__main__':
  env = E160_environment()
  graphics = E160_graphics(env)
  robot = env.robots[0]
  runRobot(env, graphics=graphics)

  sensorId = int(input("Which sensor are you calibrating? (0 - front, 1 - left, 2 - right): "))

  # robot.file_name = "Log/P"+str(DESIRED_POWER_PERCENT)+"Forward_LearningCoef"+str(learning_coefficient)+'_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S')+".txt"
  # robot.change_headers()

  for i in range(len(READING_DISTS_CM)):
    dist_cm = READING_DISTS_CM[i]
    input("Please move the center of the robot to the next location, {} cm. Press enter to collect data.".format(dist_cm))

    pos_readings = np.zeros(NUM_READINGS_PER_TEST)
    print("Collecting data...")
    for j in range(NUM_READINGS_PER_TEST):
      runRobot(env, graphics=graphics)
      pos_readings[j] = robot.range_voltages[sensorId]
    print(pos_readings)
    print("Completed collection for this distance.\n")
    readings[i,:] = pos_readings

  mean_vals = np.zeros(len(READING_DISTS_CM))
  std_devs  = np.zeros(len(READING_DISTS_CM))
  for i in range(len(READING_DISTS_CM)):
    mean_vals[i] = np.mean(np.log(readings[i,:]))
    std_devs[i]  = np.std(np.log(readings[i,:]))
  print(mean_vals, std_devs)

  # weighted linear regression of log of y values
  coefs = np.polyfit(READING_DISTS_CM, mean_vals, 1, w=1/std_devs)
  print('fit coefficients', coefs)

  # data = np.vstack(tuple([x for _,x in readings.items()]))
  df = pd.DataFrame(readings.T, columns=[str(dist)+'cm' for dist in READING_DISTS_CM])
  # TO LOG, ADD TO_CSV FUNC
  df.to_csv("/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/Log/RangeCalibrationSensor{}.csv".format(sensorId), index=False)

  ## plot everything
  fig = plt.figure()
  plt.title('Weighted semilog exponential regression for sensor {}'.format(sensorId))
  # plt.xlim(-30,30)
  # plt.ylim(-30,30)
  plt.errorbar(READING_DISTS_CM, mean_vals, yerr=std_devs, label='semilogged data')
  plt.plot(READING_DISTS_CM, coefs[0]*READING_DISTS_CM + coefs[1], label='exponential fit line')
  # plt.plot(READING_DISTS_CM[0]*np.ones(NUM_READINGS_PER_TEST), readings[20.0], label='exponential fit line')
  # plt.plot(READING_DISTS_CM[1]*np.ones(NUM_READINGS_PER_TEST), readings[30.0], label='exponential fit line')
  # plt.plot(READING_DISTS_CM[2]*np.ones(NUM_READINGS_PER_TEST), readings[40.0], label='exponential fit line')
  plt.ylabel('sensor values in [0, 255]')
  plt.xlabel('distance (cm)')
  plt.legend()
  plt.show()


