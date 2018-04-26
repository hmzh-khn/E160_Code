import math
import random
import numpy as np
from E160_config import *
from E160_state import*
from scipy.stats import norm
from scipy.linalg import sqrtm


INITIAL_ERROR = np.array([0.0, 0, 0]).reshape((3,1))

CONFIG_ROBOT_RAD_M = 0.147 / 2
CONFIG_WHEEL_RAD_M = 0.034

CONFIG_NUM_STATE_VARS = 3

# from wikipedia page of UKF
CONFIG_ALPHA = 0.001
CONFIG_BETA = 2.0
CONFIG_KAPPA = 0.0

# enums for direction
RIGHT_SENSOR_ID = 2
STRAIGHT_SENSOR_ID = 0
LEFT_SENSOR_ID = 1

print("sensor noise", CONFIG_SENSOR_NOISE_STDEV)
print("initial translation stdev", CONFIG_INIT_TRANSLATION_STDEV)
print("initial angle stdev", CONFIG_INIT_ANGLE_STDEV)

# R_t is the ``prediction noise'' - what is this?
PREDICTION_COVARIANCE = np.array([[CONFIG_SENSOR_NOISE_STDEV**2, 0, 0],
                                  [0, CONFIG_SENSOR_NOISE_STDEV**2, 0],
                                  [0, 0, CONFIG_SENSOR_NOISE_STDEV**2]]) 

# Q_t - measurement noise, for converting from sensor to absolute estimation
MEASUREMENT_COVARIANCE = np.array([[CONFIG_SENSOR_NOISE_STDEV**2, 0, 0],
                                   [0, CONFIG_SENSOR_NOISE_STDEV**2, 0],
                                   [0, 0, CONFIG_SENSOR_NOISE_STDEV**2]])

def normalize_np_angle(ang):
  ang = ang % (2 * math.pi)
  ang[ang > math.pi] -= 2 * math.pi
  return ang


class E160_UKF:

  def __init__(self, 
               environment, 
               initial_state, 
               initial_variance, 
               encoder_resolution,
               alpha=CONFIG_ALPHA,
               beta=CONFIG_BETA,
               kappa=CONFIG_KAPPA,
               robot_radius=CONFIG_ROBOT_RAD_M, 
               wheel_radius=CONFIG_WHEEL_RAD_M):
    
    # robot information
    self.robot_radius_m = robot_radius
    self.wheel_radius_m = wheel_radius

    # define the sensor orientations
    self.sensor_orientation = [0, math.pi/2, -math.pi/2] # orientations of the sensors on robot
    self.num_sensors = len(self.sensor_orientation)
    self.walls = environment.walls

    # control signal sensor (process) information
    self.encoder_resolution = encoder_resolution
    self.last_encoder_measurements = [0,0]

    # correction signal sensor (measurement) information
    self.FAR_READING = 1.5
    if CONFIG_IN_HARDWARE_MODE:
      self.IR_sigma_m = 0.2 # Range finder s.d
    else:
      self.IR_sigma_m = 0.3 # Range finder s.d

    # number of state variables
    self.num_state_vars = CONFIG_NUM_STATE_VARS

    # total number of sigma points (includes mean)
    self.numParticles = 2*self.num_state_vars + 1

    # UKF parameters
    self.alpha = alpha
    self.beta = beta
    self.kappa = kappa
    self.lmbda = None
    self.mean_weights = np.zeros(self.numParticles)
    self.cov_weights = np.zeros(self.numParticles)

    # update lambda and weights
    self.lmbda, self.gamma, self.mean_weights, self.cov_weights = self.UpdateWeights()

    print('mean weights',self.mean_weights)
    print('cov weights',self.cov_weights)

    # structures that hold data for the sigma points
    self.particles = []
    self.sigma_points = np.zeros((CONFIG_NUM_STATE_VARS, self.numParticles))

    # initialize hypothesis for state and variance
    self.state = np.array(initial_state).reshape(3,1) + INITIAL_ERROR
    self.variance = np.array(initial_variance)

    # initializes sigma_points and particles
    self.particles = self.GenerateParticles(self.state, self.variance)

    self.delay = 0

  # update weights using rules described in Probabilistic Robotics
  def UpdateWeights(self):
    """
    Create new lambda, update mean and covariance weights.
    """
    lmbda = (self.alpha **2  * (self.num_state_vars + self.kappa) 
                  - self.num_state_vars)

    gamma = np.sqrt(self.num_state_vars + lmbda)

    mean_weights = np.zeros(self.numParticles)
    cov_weights = np.zeros(self.numParticles)

    print('lambda', lmbda)
    print('gamma', gamma)
    print('num_state_vars', self.num_state_vars)

    # set weights for mean sigma point
    mean_weights[0] = lmbda / (self.num_state_vars + lmbda)
    cov_weights[0] = (lmbda / (self.num_state_vars + lmbda)
                           + 1.0 - self.alpha**2.0 + self.beta)

    # set other weights in filter
    mean_weights[1:] = 1.0 / (2 * (self.num_state_vars + lmbda))
    cov_weights[1:]  = 1.0 / (2 * (self.num_state_vars + lmbda))
    
    # mean_weights[0] = -2#1 / (self.num_state_vars + 1)
    # cov_weights[0] = -2#1 / (self.num_state_vars + 1)

    # set other weights in filter
    # mean_weights[1:] = (1.0 - mean_weights[0]) / (2 * (self.num_state_vars))
    # cov_weights[1:]  = (1.0 - cov_weights[0]) / (2 * (self.num_state_vars))

    return lmbda, gamma, mean_weights, cov_weights

  # step 2, 6 in Probabilistic Robotics - generate new sigma points
  def GenerateParticles(self, state, variance):
    """ generate sigma particles """

    numParticles = self.numParticles

    # create sigma points using the gamma offset
    sigma_offsets = np.zeros((numParticles, self.num_state_vars))
    offsets = self.gamma * np.abs(sqrtm(self.variance))

    for i in range(self.num_state_vars):
      sigma_offsets[i + 1,:] = offsets[i,:]
      sigma_offsets[self.num_state_vars + i + 1, :] = -offsets[i,:]

    # create particle object list
    particles = numParticles*[0]

    # get mean state and use it to create sigma points
    x, y, theta = state[0][0], state[1][0], state[2][0]

    for i in range(numParticles):
      particles[i] = self.UKF_Particle(x + sigma_offsets[i][0] * np.cos(theta) - sigma_offsets[i][1] * np.sin(theta),
                                       y + sigma_offsets[i][0] * np.sin(theta) + sigma_offsets[i][1] * np.cos(theta),
                                       self.normalize_angle(theta + sigma_offsets[i][2]),
                                       self.mean_weights[i])
    return particles

  # step 3 in Probabilistic Robotics
  def PropagateSigmaPoints(self, encoder_measurements, last_encoder_measurements):
    """ propagate all sigma points to get next predictions """

    for p in self.particles:
      delta_s, delta_heading = p.update_odometry(encoder_measurements,
                                                 last_encoder_measurements)
      p.update_state(delta_s, delta_heading)

  # steps 4, 5 in Probabilistic Robotics
  def PredictMeanAndCovariance(self):
    """ perform a weighted calculation of the mean and covariance of the 
        propagated sigma point predictions """

    # data processing
    particle_data = np.array([[p.x, p.y, math.cos(p.heading), math.sin(p.heading)] for p in self.particles])

    # calculate mean state
    print('particle data', particle_data)
    mean_state = np.average(particle_data, axis=0, weights=self.mean_weights)
    print('state in 4,5 a', mean_state)
    heading = np.arctan2(mean_state[3], mean_state[2])
    mean_state = np.array([mean_state[0], mean_state[1], heading]).reshape(3,1)
    print('state in 4,5 b', mean_state)

    particle_data[:,2] = np.arctan2(particle_data[:,3], particle_data[:,2])
    particle_data = particle_data[:,0:3]
    # print('particle_data', particle_data)
    # print('predicted mean state\n', mean_state)

    # calculate new variance
    variance = np.zeros((self.num_state_vars, self.num_state_vars))
    for i in range(self.numParticles):
      particle_data_vec = particle_data[i,:].reshape((3,1))
      error = (particle_data_vec - mean_state)
      # print(i, 'particle data row\n', particle_data[i,:])
      # print(i, 'error vector\n', error, error.shape)
      # normalize heading
      error[2] = self.normalize_angle(error[2])
      #print('error vector w/ normalized angle', error)
      variance = variance + np.dot(self.cov_weights[i], 
                                   np.dot(error,
                                          np.transpose(error)))
      # print(i, 'th variance matrix (no pred. matrix)\n', variance)
    variance = variance + PREDICTION_COVARIANCE
    # print('variance  matrix (with pred. matrix)\n', variance)

    return mean_state.reshape(3,1), variance

  # step 7 in Probabilistic Robotics
  def CalculateExpectedMeasurements(self):
    """ gets the expected measurements for all the sigma points """

    expected_measurements_m = np.zeros((self.numParticles, self.num_sensors))

    for i in range(self.numParticles):
      # x, y, theta = self.particles[i].x, self.particles[i].y, self.particles[i].heading

      min_dist_right = min(self.FindMinWallDistance(self.particles[i], self.walls, self.sensor_orientation[RIGHT_SENSOR_ID]), self.FAR_READING)
      min_dist_straight = min(self.FindMinWallDistance(self.particles[i], self.walls, self.sensor_orientation[STRAIGHT_SENSOR_ID]), self.FAR_READING)
      min_dist_left = min(self.FindMinWallDistance(self.particles[i], self.walls, self.sensor_orientation[LEFT_SENSOR_ID]), self.FAR_READING)

      expected_measurements_m[i][RIGHT_SENSOR_ID] = min_dist_right
      expected_measurements_m[i][STRAIGHT_SENSOR_ID] = min_dist_straight
      expected_measurements_m[i][LEFT_SENSOR_ID] = min_dist_left

    return expected_measurements_m

  # step 8, 9 in Probabilistic Robotics
  def SensorMeanAndCovariance(self, expected_measurements_m):
    """ get mean and covariance of sensor measurements """
    # calculate mean expected sensor measurements
    expected_measurement_mean = np.average(expected_measurements_m, 
                                            axis=0, 
                                            weights=self.mean_weights).reshape((3,1))

    # calculate new expected measurement variance
    variance = np.zeros((self.num_state_vars, self.num_state_vars))
    for i in range(self.numParticles):
      error = expected_measurements_m[i,:].reshape((3,1)) - expected_measurement_mean
      variance = variance + self.cov_weights[i] * np.dot(error, np.transpose(error))

    expected_measurement_variance = variance + MEASUREMENT_COVARIANCE

    return expected_measurement_mean, expected_measurement_variance

  # step 10 in Probabilistic Robotics
  # TODO: Does not change currently as a function of orientation?
  def CalculateCrossCovariance(self,
                               particles,
                               state,
                               expected_measurements_m,
                               expected_measurement_mean):
    """ calculate cross covariance between measurement estimates and state prediction """

    # calculate cross variance
    cross_covariance = np.zeros((self.num_state_vars, self.num_state_vars))
    particle_data = np.array([[p.x, p.y, p.heading] for p in self.particles])
    for i in range(self.numParticles):
      state_error = particle_data[i,:].reshape((3,1)) - state
      print(i, 'state error, mean state', state_error, state)
      state_error[2] = self.normalize_angle(state_error[2])

      exp_measurement_error = (expected_measurements_m[i,:].reshape((3,1)) - expected_measurement_mean).reshape((3,1))
      cross_covariance = (cross_covariance 
                          + np.dot(self.cov_weights[i], 
                                   np.dot((state_error),
                                          (np.transpose(exp_measurement_error)))))
      print(i, 'cross covariance', cross_covariance)

    return cross_covariance


  def LocalizeEst(self, 
                  encoder_measurements, 
                  last_encoder_measurements, 
                  sensor_readings):
    ''' Localize the robot with particle filters. Call everything
      Args: 
        delta_s (float): change in distance as calculated by odometry
        delta_heading (float): change in heading as calculated by odometry
        sensor_readings([float, float, float]): sensor readings from range fingers
      Return:
        None'''

    if self.delay % 15 == 5:
      raise Exception('third step')
    self.delay += 1

    # convert sensor readings to distances (with max of 1.5m)
    sensor_readings = np.array([min(reading, self.FAR_READING) for reading in sensor_readings]).reshape((3,1))

    print('before 2 state', self.state)
    # step 2 - identify sigma points at t-1
    self.particles = self.GenerateParticles(self.state, self.variance)
    print('after 2 state', self.state)

    # step 3 - propagate set of sigma points
    self.PropagateSigmaPoints(encoder_measurements, last_encoder_measurements)
    print('after 3 state', self.state)

    # step 4, 5 - calculate weighted means and covariance of sigma points
    # this step is already complete in step 1 since all weights are equal
    var1 = self.variance
    self.state, self.variance = self.PredictMeanAndCovariance()
    print('before 6 state', self.state)
    # print('variance before -> after\n', var, '\n', self.variance)

    if True:
      # step 6 - identify sigma points at time t using predicted mean, covariance
      self.particles = self.GenerateParticles(self.state, self.variance)

      # step 7 - calculate the expected sensor measurements
      expected_measurements_m = self.CalculateExpectedMeasurements()

      # step 8, 9 - calculate mean and variance of expected sensor measurements
      exp_measurement_mean, exp_measurement_variance = self.SensorMeanAndCovariance(expected_measurements_m)
      # print('exp measurement variance', exp_measurement_variance)

      # step 10 - calculate cross-covariance between predicted state and measurements
      cross_covariance = self.CalculateCrossCovariance(self.particles, 
                                                       self.state, 
                                                       expected_measurements_m, 
                                                       exp_measurement_mean)
      # print('cross variance, \nstate same across row (x, y, theta), \nmeasurement (straight, left, right) same across column \n', cross_covariance)

      # step 11 - calculate Kalman gain
      print('exp_measurement_variance inverse\n', np.linalg.inv(exp_measurement_variance))
      kalman_gain = np.dot(cross_covariance, np.linalg.inv(exp_measurement_variance))
      print('kalman gain', kalman_gain)

      # step 12,13 - use actual measurements to calculate new state estimate, covariance
      print('sensor v. expected sensor mean', sensor_readings, exp_measurement_mean)
      innovation = sensor_readings - exp_measurement_mean
      print('innovation, delta state', innovation, np.dot(kalman_gain, innovation))
      print('state before innovate', self.state)

      self.state = self.state + np.dot(kalman_gain, innovation)
      print('state after innovate', self.state)
      var2 = self.variance
      self.variance = self.variance - np.dot(kalman_gain, np.dot(exp_measurement_variance, np.linalg.inv(kalman_gain)))

      print('variance before -> after -> after correction \n', var1, '\n', var2, '\n', self.variance)

      print(self.delay, 'over \n\n\n\n')

    state = E160_state(self.state[0][0], self.state[1][0], self.state[2][0])
    print('palmer oops?', state,'\n', self.state[1][0])
    return state



  def FindMinWallDistance(self, particle, walls, sensorT):
    ''' Given a particle position, walls, and a sensor, find 
      shortest distance to the wall
      Args:
        particle (E160_Particle): a particle 
        walls ([E160_wall, ...]): represents endpoint of the wall 
        sensorT: orientation of the sensor on the robot
      Return:
        distance to the closest wall' (float)'''

    #Handle the off centeritude of the horizontal sensors
    sensor_vertical_offset = 0 #straight sensor or in simulation
    if CONFIG_IN_HARDWARE_MODE(CONFIG_ROBOT_MODE):
      if sensorT > 0.1: #Left sensor
        sensor_vertical_offset = CONFIG_LEFT_VERTICAL_OFFSET
      elif sensorT < - 0.1:
        sensor_vertical_offset = CONFIG_RIGHT_VERTICAL_OFFSET

    temp_particle = self.UKF_Particle(0.0,0.0,particle.heading,1.0/self.numParticles)
    temp_particle.x = particle.x + math.cos(particle.heading) * sensor_vertical_offset
    temp_particle.y = particle.y + math.sin(particle.heading) * sensor_vertical_offset

    return min([self.FindWallDistance(particle, wall, sensorT) for wall in walls])
    
  def FindWallDistance(self, particle, wall, sensorT):
    ''' Given a particle position, a wall, and a sensor, find distance to the wall
      Args:
        particle (E160_Particle): a particle 
        wall ([float x4]): represents endpoint of the wall 
        sensorT: orientation of the sensor on the robot
      Return:
        distance to the closest wall (float)'''
    sensor_heading = self.normalize_angle(particle.heading + sensorT)
    sensor_slope = math.tan(sensor_heading)
    sensor_intercept = particle.y - sensor_slope * particle.x
    wall_slope, wall_intercept = wall.slope_intercept()

    # if wall slope parallel to sensor slope, sensor will not sense wall
    if wall_slope == sensor_slope:
      return float('inf')

    slope_diff = sensor_slope - wall_slope
    intercept_diff = wall_intercept - sensor_intercept
    x_val = intercept_diff/slope_diff

    # if wall is perfectly vertical, x_vale must be same as wall x_val
    if abs(wall_slope) > 1000:
      x_val, _ =  wall.point1

    # if sensor line is vertical, intersection will be at this x_point
    if abs(sensor_slope) > 1000:
      x_val = particle.x
      y_val = wall_slope * x_val + wall_intercept
    else:
      y_val = sensor_slope * x_val + sensor_intercept
    point = (x_val, y_val)
    # ensure that sensor points towards wall (not other direction)


    if(wall.contains_point(point)):
      distance = math.sqrt((x_val-particle.x)**2 + (y_val-particle.y)**2)
      if abs(math.atan2(particle.y - y_val, particle.x - x_val) - sensor_heading) >= CONFIG_HEADING_TOLERANCE:
        return float('inf')
      return distance
    else:
      return float('inf')

  def normalize_angle(self, ang):
    ''' Wrap angles between -pi and pi'''
    ang = ang % (2 * math.pi)
    while ang < -math.pi:
      ang = ang + 2 * math.pi
    while ang > math.pi:
      ang = ang - 2 * math.pi
    return ang

  class UKF_Particle:
    def __init__(self, x, y, heading, weight):
      self.x = x
      self.y = y
      self.heading = heading
      self.weight = weight
      self.is_first_run = False
      self.weight_memory = 20
      self.recent_weights = [0] * self.weight_memory

    def __str__(self):
      return str(self.x) + " " + str(self.y) + " " + str(self.heading) + " " + str(self.weight)

    # clean up this function
    def update_odometry(self, encoder_measurements, last_encoder_measurements):

      delta_s = 0
      delta_heading = 0

      left_encoder_measurement = encoder_measurements[0]
      right_encoder_measurement = encoder_measurements[1] 

      last_left_encoder_measurement = last_encoder_measurements[0]
      last_right_encoder_measurement = last_encoder_measurements[1]

      # print(left_encoder_measurement, last_left_encoder_measurement)#, right_encoder_measurement - last_right_encoder_measurement)

      delta_left =  float(left_encoder_measurement - last_left_encoder_measurement) #* rands[0]
      delta_right = float(right_encoder_measurement - last_right_encoder_measurement) #* rands[1]

      # print('delta-dir', delta_left, delta_right)

      if self.is_first_run:
          delta_right = 0
          delta_left = 0
          self.is_first_run = False

      # cause the lab said so I like my name better
      diffEncoder0 = delta_left
      diffEncoder1 = delta_right

      wheel_circumference = 2 * math.pi * CONFIG_WHEEL_RAD_M


      # TODO: implement calibration from ticks to centimeters
      # left_distance = (delta_left / self.encoder_resolution) * wheel_circumference
      # right_distance = (delta_right / self.encoder_resolution) * wheel_circumference
      left_distance  = delta_left  * CONFIG_CM_TO_M *  CONFIG_LEFT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10]
      right_distance = delta_right * CONFIG_CM_TO_M * CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10]


      delta_s = (left_distance + right_distance) / 2
      delta_heading = (right_distance - left_distance) / (2 * CONFIG_ROBOT_RAD_M)


      # set current measurements as the last for next cycle - happens in E160_robot.py
      # self.last_encoder_measurements[0] = left_encoder_measurement
      # self.last_encoder_measurements[1] = right_encoder_measurement
          
      # keep this to return appropriate changes in distance, angle
      return delta_s, delta_heading 

    def update_state(self, delta_s, delta_heading):

      self.x = self.x + math.cos(self.heading + delta_heading / 2) * delta_s
      
      self.y = self.y + math.sin(self.heading + delta_heading / 2) * delta_s

      self.heading = self.normalize_angle(self.heading + delta_heading)

    def normalize_angle(self, ang):
      ''' Wrap angles between -pi and pi'''
      while ang < -math.pi:
        ang = ang + 2 * math.pi
      while ang > math.pi:
        ang = ang - 2 * math.pi
      return ang

