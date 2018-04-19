import math
import random
import numpy as np
import copy
from E160_config import *
from E160_state import*
from scipy.stats import norm

CONFIG_RANDOM_START = True
if CONFIG_RANDOM_START:
  CONFIG_GAUSS_MULT = 0.1
else:
  CONFIG_GAUSS_MULT = 0.2
CONFIG_ROBOT_RAD_M = 0.147 / 2
CONFIG_WHEEL_RAD_M = 0.034
CONFIG_DELETE_PARTICLE_THRESHOLD = 1.0/2

CONFIG_SENSOR_NOISE = 0.0

CONFIG_NUM_STATE_VARS = 3

class E160_UKF:

  def __init__(self, environment, initial_state, initial_variance, robotWidth, wheel_radius, encoder_resolution):
    self.particles = []
    self.environment = environment
    self.numParticles = 2*CONFIG_NUM_STATE_VARS + 1

    self.state = initial_state
    self.variance = initial_variance
    self.GenerateParticles()
    
    # maybe should just pass in a robot class?
    self.robotWidth = robotWidth
    self.radius = robotWidth/2
    self.wheel_radius = wheel_radius
    self.encoder_resolution = encoder_resolution
    self.FAR_READING = 1.5
    
    # PF parameters
    if CONFIG_IN_HARDWARE_MODE:
      self.IR_sigma_m = 0.2 # Range finder s.d
    else:
      self.IR_sigma_m = 0.3 # Range finder s.d
    self.odom_xy_sigma = 1.25 # odometry delta_s s.d
    self.odom_heading_sigma = 0.75  # odometry heading s.d
    # self.particle_weight_sum = 0

    # define the sensor orientations
    self.sensor_orientation = [-math.pi/2, 0, math.pi/2] # orientations of the sensors on robot
    self.walls = self.environment.walls

    # initialize the current state
    self.state = E160_state()
    self.state.set_state(initial_state)

    # TODO: change this later
    self.map_maxX = CONFIG_MAP_MAX_X_M
    self.map_minX = CONFIG_MAP_MIN_X_M
    self.map_maxY = CONFIG_MAP_MAX_Y_M
    self.map_minY = CONFIG_MAP_MIN_Y_M

    # self.InitializeParticles()
    self.last_encoder_measurements =[0,0]

  def GenerateParticles(self):
    ''' Populate self.particles with random Particle 
      Args:
        None
      Return:
        None'''

    self.sigma_offsets = np.zeros((CONFIG_NUM_STATE_VARS, self.numParticles))
    for i in range(len(self.variance)):
      stdev = np.sqrt(self.variance[i][i])

      # sets the negative offset for each state variable
      self.sigma_offsets[i][i] = -stdev
      # sets the positive offset for each state variable
      self.sigma_offsets[i][i + CONFIG_NUM_STATE_VARS + 1] = stdev


    self.particles = self.numParticles*[0]
    # global state
    x, y, theta = self.state.x, self.state.y, self.state.theta

    for i in range(0, self.numParticles):
        self.particles[i] = self.UKF_Particle(x + self.sigma_offsets[0][i] * np.cos(theta) - self.sigma_offsets[1][i] * np.sin(theta),
                                          y + self.sigma_offsets[0][i] * np.sin(theta) + self.sigma_offsets[1][i] * np.cos(theta),
                                          self.normalize_angle(theta + self.sigma_offsets[2][i]),
                                          1.0/self.numParticles)
            
  def LocalizeEstWithParticleFilter(self, 
                                    encoder_measurements, 
                                    last_encoder_measurements, 
                                    sensor_readings):
    ''' Localize the robot with particle filters. Call everything
      Args: 
        delta_s (float): change in distance as calculated by odometry
        delta_heading (float): change in heading as calcualted by odometry
        sensor_readings([float, float, float]): sensor readings from range fingers
      Return:
        None'''

    # convert sensor readings to distances (with max of 1.5m)
    sensor_readings = [min(reading, self.FAR_READING) for reading in sensor_readings]
    self.GenerateParticles()

    # propagate the particles with the model
    total_weight = 0
    for i in range(self.numParticles):
      if encoder_measurements != last_encoder_measurements:
        rands = [random.gauss(1,CONFIG_SENSOR_NOISE),random.gauss(1,CONFIG_SENSOR_NOISE)]

        self.particles[i].weight = self.CalculateWeight(sensor_readings, self.walls, self.particles[i]) 
        self.Propagate(encoder_measurements, last_encoder_measurements, i, rands)

        #TODO NOT JUST RANDOMLY PROPOGATE AFTER CALCULATING WEIGHT
        # The issue: Min dist straight is one cycle ahead of the sensor readings...

      total_weight += self.particles[i].weight

    print('weights', [p.weight for p in self.particles])

    for i in range(self.numParticles):
      self.particles[i].weight = self.particles[i].weight / total_weight

    

    ### TODO: START HERE
    # self.UpdateVariance()
    

    return self.GetEstimatedPos()

  def Propagate(self, encoder_measurements, last_encoder_measurements, i, rands):
    '''Propagate all the particles from the last state with odometry readings
      Args:
        delta_s (float): distance traveled based on odometry
        delta_heading(float): change in heading based on odometry
      return:
        nothing'''

    delta_s, delta_heading = self.particles[i].update_odometry(encoder_measurements, 
                                                               last_encoder_measurements, rands)
    self.particles[i].update_state(delta_s, delta_heading)
        
  def CalculateWeight(self, sensor_readings, walls, particle):
    '''Calculate the weight of a particular particle
      Args:
        particle (E160_Particle): a given particle
        sensor_readings ( [float, ...] ): readings from the IR sensors
        walls ([ [four doubles], ...] ): positions of the walls from environment, 
              represented as 4 doubles 
      return:
        new weight of the particle (float) '''

    newWeight = 0
    
    # upper bound the distance at 1000 m
    min_dist_right = min(self.FindMinWallDistance(particle, walls, self.sensor_orientation[0]), self.FAR_READING)
    min_dist_straight = min(self.FindMinWallDistance(particle, walls, self.sensor_orientation[1]), self.FAR_READING)
    min_dist_left = min(self.FindMinWallDistance(particle, walls, self.sensor_orientation[2]), self.FAR_READING)

    error_right = min_dist_right - sensor_readings[2]
    error_straight = min_dist_straight - sensor_readings[0]
    error_left = min_dist_left - sensor_readings[1]

    error = ((error_right)**2
              + (error_straight)**2
              + (error_left**2))

    print('error', error_left, error_straight, error_right, min_dist_straight, sensor_readings[0])

    prob = math.exp(-error/self.IR_sigma_m**2)
    #print(prob)
    #print(["%0.2f" % i for i in [min_dist_right, min_dist_straight, min_dist_left, particle.x, particle.y, prob]])

    # make weights this nonzero
    return prob

  def GetEstimatedPos(self):
    ''' Calculate the mean of the particles and return it 
      Args:
        None
      Return:
        None'''
    arr = np.array([[p.x, p.y, math.cos(p.heading), math.sin(p.heading)] for p in self.particles])
    weights = np.array([p.weight for p in self.particles])

     # identify new state
    means = np.average(arr, axis=0, weights=weights)

    # Fast and numerically precise:
    weighted_variance = np.average((arr-means)**2, weights=weights, axis=0)
    print(np.sqrt(weighted_variance))
    weighted_std = np.sqrt(weighted_variance)
    self.variance[0][0] = weighted_variance[0]
    self.variance[1][1] = weighted_variance[1]
    cos_variance = weighted_variance[2]*1/(1+(means[3]/means[2])**2)* \
                      means[3]*1/(means[2]**2)
    sin_variance = weighted_variance[3]*1/(1+(means[3]/means[2])**2)* \
                      1/means[2]
    theta_variance = math.sqrt(cos_variance**2 + sin_variance**2)
    self.variance[2][2] = theta_variance

    heading = math.atan2(means[3], means[2])

    self.state.set_state(means[0], means[1], heading)

    print('state actual', str(self.environment.robots[0].state_odo))
    print('state ukf', str(self.state))

    return self.state

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
    def update_odometry(self, encoder_measurements, last_encoder_measurements, rands):

      delta_s = 0
      delta_heading = 0

      left_encoder_measurement = encoder_measurements[0]
      right_encoder_measurement = encoder_measurements[1] 

      last_left_encoder_measurement = last_encoder_measurements[0]
      last_right_encoder_measurement = last_encoder_measurements[1]

      # print(left_encoder_measurement, last_left_encoder_measurement)#, right_encoder_measurement - last_right_encoder_measurement)

      delta_left =  float(left_encoder_measurement - last_left_encoder_measurement) * rands[0]
      delta_right = float(right_encoder_measurement - last_right_encoder_measurement) * rands[1]

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
