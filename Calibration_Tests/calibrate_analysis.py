import sys
sys.path.insert(0, "/Users/hikhan/Desktop/Autonomous Robotics Navigation/E160_Code/")
sys.path.insert(0, "/Users/Loaner/Documents/E160_Code")
import matplotlib
# print(matplotlib.get_backend())
matplotlib.use("TkAgg")

#from E160_config import CONFIG_DELTA_T
#from E160_environment import *
#from E160_graphics import *
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

sensorId = 0

readings = pd.read_csv("LabData/RangeCalibrationSensor{}.csv".format(sensorId), index_col=0)

READING_DISTS_CM = np.array([10, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80, 90, 100, 120, 140])
if sensorId == 0:
  readings = readings.drop(["120cm"], axis=1)
  READING_DISTS_CM = np.array([10, 15, 20, 25, 30, 35, 40, 50, 60, 70, 80, 90, 100, 140])

mean_vals = np.zeros(len(READING_DISTS_CM))
std_devs  = np.zeros(len(READING_DISTS_CM))
mean_vals = np.mean(np.log(readings), axis=0)
std_devs  = np.std(np.log(readings), axis=0)
# for i in range(len(READING_DISTS_CM)):
#   mean_vals[i] = np.mean(np.log(readings[:,i]))
#   std_devs[i]  = np.std(np.log(readings[:,i]))
print(mean_vals, std_devs)

# weighted linear regression of log of y values
coefs1 = np.polyfit(READING_DISTS_CM[2:10], mean_vals[2:10], 1, w=1/std_devs[2:10])
if sensorId == 0:
  coefs2 = np.polyfit(READING_DISTS_CM[9:], mean_vals[9:], 1)
else:
  coefs2 = np.polyfit(READING_DISTS_CM[9:], mean_vals[9:], 1, w=1/std_devs[9:])
print('fit coefficients 1 sensor: ',sensorId,' : ', coefs1)
print('fit coefficients 2 sensor: ',sensorId,' : ', coefs2)

# data = np.vstack(tuple([x for _,x in readings.items()]))
# TO LOG, ADD TO_CSV FUNC

## plot everything
fig = plt.figure()
plt.title('Weighted semilog exponential regression for sensor {}'.format(sensorId))
# plt.xlim(-30,30)
# plt.ylim(-30,30)
plt.errorbar(READING_DISTS_CM, mean_vals, yerr=std_devs, label='semilogged data')
plt.plot(READING_DISTS_CM[2:10], coefs1[0]*READING_DISTS_CM[2:10] + coefs1[1], label='exponential fit line')
plt.plot(READING_DISTS_CM[9:], coefs2[0]*READING_DISTS_CM[9:] + coefs2[1], label='exponential fit line')
# plt.plot(READING_DISTS_CM[0]*np.ones(NUM_READINGS_PER_TEST), readings[20.0], label='exponential fit line')
# plt.plot(READING_DISTS_CM[1]*np.ones(NUM_READINGS_PER_TEST), readings[30.0], label='exponential fit line')
# plt.plot(READING_DISTS_CM[2]*np.ones(NUM_READINGS_PER_TEST), readings[40.0], label='exponential fit line')
plt.ylabel('sensor values in [0, 255]')
plt.xlabel('distance (cm)')
plt.legend()
plt.show()
