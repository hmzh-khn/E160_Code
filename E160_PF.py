import math
import random
import numpy as np
import copy
from E160_config import *
from E160_state import*
from scipy.stats import norm


CONFIG_GAUSS_MULT = 0.05
CONFIG_ROBOT_RAD_M = 0.147 / 2
CONFIG_WHEEL_RAD_M = 0.034

class E160_PF:

  def __init__(self, environment, robotWidth, wheel_radius, encoder_resolution):
    self.particles = []
    self.environment = environment
    self.numParticles = 3
    
    # maybe should just pass in a robot class?
    self.robotWidth = robotWidth
    self.radius = robotWidth/2
    self.wheel_radius = wheel_radius
    self.encoder_resolution = encoder_resolution
    self.FAR_READING = 1000
    
    # PF parameters
    self.IR_sigma = 0.2 # Range finder s.d
    self.odom_xy_sigma = 1.25 # odometry delta_s s.d
    self.odom_heading_sigma = 0.75  # odometry heading s.d
    self.particle_weight_sum = 0

    # define the sensor orientations
    self.sensor_orientation = [-math.pi/2, 0, math.pi/2] # orientations of the sensors on robot
    self.walls = self.environment.walls

    # initialize the current state
    self.state = E160_state()
    self.state.set_state(0,0,0)

    # TODO: change this later
    self.map_maxX = 1.0
    self.map_minX = -1.0
    self.map_maxY = 1.0
    self.map_minY = -1.0
    self.known_start = self.Particle(0,0,0,1.0/self.numParticles)

    self.InitializeParticles()
    self.last_encoder_measurements =[0,0]

    

  def InitializeParticles(self):
    ''' Populate self.particles with random Particle 
      Args:
        None
      Return:
        None'''
    self.particles = self.numParticles*[0]
    for i in range(0, self.numParticles):
      #self.SetRandomStartPos(i)
      self.SetKnownStartPos(i)
      self.particles[i].is_first_run = True

  def SetRandomStartPos(self, i):
    x_naught = random.random(self.map_minX, self.map_maxX)
    y_naught = random.random(self.map_minY, self.map_maxY)
    self.particles[i] = self.Particle(x_naught, y_naught, random.random(-math.pi,math.pi) ,1.0/self.numParticles)

  def SetKnownStartPos(self, i):
    self.particles[i] = self.known_start
            
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

    # print(encoder_measurements[0], last_encoder_measurements[0])

    # randomly propagate the movement
    print('----')
    for i in range(self.numParticles):
      self.Propagate(encoder_measurements, last_encoder_measurements, i)
        
        
    return self.GetEstimatedPos()

  def Propagate(self, encoder_measurements, last_encoder_measurements, i):
    '''Propagate all the particles from the last state with odometry readings
      Args:
        delta_s (float): distance traveled based on odometry
        delta_heading(float): change in heading based on odometry
      return:
        nothing'''

    delta_s, delta_heading = self.particles[i].update_odometry(encoder_measurements, 
                                                               last_encoder_measurements)
    self.particles[i].update_state(delta_s, delta_heading)

        
  def CalculateWeight(self, sensor_readings, walls, particle):
    '''Calculate the weight of a particular particle
      Args:
        particle (E160_Particle): a given particle
        sensor_readings ( [float, ...] ): readings from the IR sesnors
        walls ([ [four doubles], ...] ): positions of the walls from environment, 
              represented as 4 doubles 
      return:
        new weight of the particle (float) '''

    newWeight = 0
        # add student code here 
        
        
        
        # end student code here
    return newWeight

  def Resample(self):
    '''Resample the particles systematically
      Args:
        None
      Return:
        None'''
        # add student code here 
        
        
        
        # end student code here

  def GetEstimatedPos(self):
    ''' Calculate the mean of the particles and return it 
      Args:
        None
      Return:
        None'''
        # add student code here 
        
        
        
        # end student code here
        
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
    if abs(wall_slope) == abs(sensor_slope):
      return float('inf')

    slope_diff = sensor_slope - wall_slope
    intercept_diff = wall_intercept - sensor_intercept
    x_val = intercept_diff/slope_diff

    # if wall is perfectly vertical, x_vale must be same as wall x_val
    if abs(wall_slope) == float('inf'):
      x_val, _ =  wall.point1

    # if sensor line is vertical, intersection will be at this x_point
    if abs(sensor_slope) == float('inf'):
      x_val = particle.x
      y_val = wall_slope * x_val + wall_intercept
    else:
      y_val = sensor_slope * x_val + sensor_intercept
    point = (x_val, y_val)

    # ensure that sensor points towards wall (not other direction)
    if abs(math.atan2(particle.y - y_val, particle.x - x_val) - sensor_heading) >= CONFIG_HEADING_TOLERANCE:
      return float('inf')

    if(wall.contains_point(point)):
      distance = math.sqrt((x_val-particle.x)**2 + (y_val-particle.y)**2)
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

  class Particle:
    def __init__(self, x, y, heading, weight):
      self.x = x
      self.y = y
      self.heading = heading
      self.weight = weight

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

      rand1 = random.gauss(1.0, CONFIG_GAUSS_MULT)
      rand2 = random.gauss(1.0, CONFIG_GAUSS_MULT)

      # print(left_encoder_measurement, last_left_encoder_measurement)#, right_encoder_measurement - last_right_encoder_measurement)

      delta_left =  (float(left_encoder_measurement - last_left_encoder_measurement)
                     * rand1)
      delta_right = (float(right_encoder_measurement - last_right_encoder_measurement)
                     * rand2)

      print(delta_left, delta_right)

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

      print(delta_s, delta_heading)
      self.x = self.x + math.cos(self.heading + delta_heading / 2) * delta_s
      self.y = self.y + math.sin(self.heading + delta_heading / 2) * delta_s

      print(self.x, self.y, self.heading)

      self.heading = self.normalize_angle(self.heading + delta_heading)

    def normalize_angle(self, ang):
      ''' Wrap angles between -pi and pi'''
      while ang < -math.pi:
        ang = ang + 2 * math.pi
      while ang > math.pi:
        ang = ang - 2 * math.pi
      return ang