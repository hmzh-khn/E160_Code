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


df = pd.DataFrame(readings.T, columns=[str(dist)+'cm' for dist in READING_DISTS_CM])
df.to_csv("../../Log/RangeCalibrationSensor{}.csv".format(0))
READING_DISTS_CM = df
mean_vals = np.zeros(len(READING_DISTS_CM))
std_devs  = np.zeros(len(READING_DISTS_CM))
for i in range(len(READING_DISTS_CM)):
  mean_vals[i] = np.mean(np.log(readings[i,:]))
  std_devs[i]  = np.std(np.log(readings[i,:]))
print(mean_vals, std_devs)

# weighted linear regression of log of y values
coefs1 = np.polyfit(READING_DISTS_CM[2:9], mean_vals[2:9], 1, w=1/std_devs[2:9])
coefs2 = np.polyfit(READING_DISTS_CM[9:], mean_vals[9:], 1, w=1/std_devs[9:])
print('fit coefficients', coefs1)

# data = np.vstack(tuple([x for _,x in readings.items()]))
# TO LOG, ADD TO_CSV FUNC

## plot everything
fig = plt.figure()
plt.title('Weighted semilog exponential regression for sensor {}'.format(sensorId))
# plt.xlim(-30,30)
# plt.ylim(-30,30)
plt.errorbar(READING_DISTS_CM, mean_vals, yerr=std_devs, label='semilogged data')
plt.plot(READING_DISTS_CM[2:9], coefs[0]*READING_DISTS_CM[2:9] + coefs[1], label='exponential fit line')
plt.plot(READING_DISTS_CM[9:], coefs[0]*READING_DISTS_CM[9:] + coefs[1], label='exponential fit line')
# plt.plot(READING_DISTS_CM[0]*np.ones(NUM_READINGS_PER_TEST), readings[20.0], label='exponential fit line')
# plt.plot(READING_DISTS_CM[1]*np.ones(NUM_READINGS_PER_TEST), readings[30.0], label='exponential fit line')
# plt.plot(READING_DISTS_CM[2]*np.ones(NUM_READINGS_PER_TEST), readings[40.0], label='exponential fit line')
plt.ylabel('sensor values in [-255, 255]')
plt.xlabel('distance (cm)')
plt.legend()
plt.show()