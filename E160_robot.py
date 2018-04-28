
from E160_config import *
from E160_state import *
from E160_PF import *
from E160_UKF1 import *
import math
import datetime
import time
import random

CONFIG_PARTICLE_FILTER = "PF"
CONFIG_UNSCENTED_KALMAN_FILTER = "UKF"

CONFIG_FILTER = CONFIG_UNSCENTED_KALMAN_FILTER

TEST_PATH_1 = [E160_state(0,0,0), 
               E160_state(0.25,0,0), 
               E160_state(-0.3,0, math.pi),
               E160_state(-0.3,0.5, math.pi),
               E160_state(-0.1,-0.1, math.pi)]

INDOOR_TEST_PATH_1 = [E160_state(10.25, 9, 0),
                      E160_state(10.25, 9, CONFIG_QUARTER_TURN),
                      E160_state(13.5, 34, CONFIG_QUARTER_TURN),
                      E160_state(13.5, 34, 3*CONFIG_EIGHTH_TURN),
                      E160_state(25, 41.5, 3*CONFIG_EIGHTH_TURN),
                      E160_state(25, 41.5, 5*CONFIG_EIGHTH_TURN),
                      E160_state(36.5, 28.125, 5*CONFIG_EIGHTH_TURN),
                      E160_state(38.5, 11.5, -CONFIG_QUARTER_TURN),
                      E160_state(38.5, 11.5, 5*CONFIG_EIGHTH_TURN),
                      E160_state(60.5, 12, CONFIG_HALF_TURN),]

DEBUG_FORWARD_PATH_1 = [E160_state(10.25, 34, math.pi/2)]
                      # E160_state(13.5, 34, CONFIG_QUARTER_TURN),
                      # E160_state(13.5, 34, 3*CONFIG_EIGHTH_TURN),
                      # E160_state(25, 41.5, 3*CONFIG_EIGHTH_TURN),
                      # E160_state(25, 41.5, 5*CONFIG_EIGHTH_TURN),
                      # E160_state(36.5, 28.125, 5*CONFIG_EIGHTH_TURN),
                      # E160_state(38.5, 11.5, -CONFIG_QUARTER_TURN),
                      # E160_state(38.5, 11.5, 5*CONFIG_EIGHTH_TURN),
                      # E160_state(60.5, 12, CONFIG_HALF_TURN),]

STD_PATH = TEST_PATH_1#[E160_state(36*CONFIG_IN_TO_M,0,0)]

CONFIG_ROBOT_PATH = INDOOR_TEST_PATH_1
[s.set_state((CONFIG_IN_TO_M * s.x) - 0.5, - (CONFIG_IN_TO_M * s.y) + 0.5, s.theta) for s in CONFIG_ROBOT_PATH]


class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment

        # state estimation and destination
        if CONFIG_COURSE != INDOOR_COURSE:
            x0, y0, t0 = 0, 0, 0
            self.state_est = E160_state()
            self.state_est.set_state(0,0,0)
            self.state_des = E160_state()
            self.state_des.set_state(0,0,0)
            self.state_draw = E160_state()
            self.state_draw.set_state(0,0,0) 
            self.state_odo = E160_state()
            self.state_odo.set_state(0,0,0)
            self.state_ctrl = E160_state()
            self.state_odo.set_state(0,0,0)

        else:
            x0, y0, t0 = (10.25-CONFIG_M_TO_IN/2)*CONFIG_IN_TO_M, (-9+CONFIG_M_TO_IN/2)*CONFIG_IN_TO_M, math.pi/2
            self.state_est = E160_state()
            self.state_est.set_state(x0, y0, t0)
            self.state_des = E160_state()
            self.state_des.set_state(x0, y0, t0)
            self.state_draw = E160_state()
            self.state_draw.set_state(x0, y0, t0) 
            self.state_odo = E160_state()
            self.state_odo.set_state(x0, y0, t0)
            self.state_ctrl = E160_state()
            self.state_odo.set_state(x0, y0, t0)

        self.state_ukf = E160_state()
        self.state_ukf.set_state(x0, y0, t0)

        self.R = 0
        self.L = 0
        self.radius = 0.147 / 2
        self.width = 2*self.radius
        self.wheel_radius = 0.034
        self.address = address
        self.ID = self.address.encode().__str__()[-1]
        self.last_measurements = []
        self.robot_id = robot_id
        self.manual_control_left_motor = 0
        self.manual_control_right_motor = 0
        self.file_name = 'Log/Bot' + str(self.robot_id) + '_' + datetime.datetime.now().replace(microsecond=0).strftime('%y-%m-%d %H.%M.%S') + '.txt'
        self.make_headers()
        # self.encoder_resolution = 1440
        
        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0

        self.delta_right = 0
        self.delta_left = 0

        self.is_first_run = True

        # stores changes in deltaS, deltaTheta between iterations
        self.delta_state_odo = (0, 0)
        self.testing_power_L = 0
        self.testing_power_R = 0

        # Lab 3
        # point tracking
        # speedy scales parameters
        if CONFIG_IN_SIMULATION_MODE(self.environment.robot_mode):
            # self.speedy = 1.0
            self.K_rho = 0.005#1.0
            self.K_alpha = 0.04#2.0
            self.K_beta =  -0.1#-0.5

        if CONFIG_IN_HARDWARE_MODE(self.environment.robot_mode):
            # self.speedy = 0.1
            self.K_rho = 1.0 #* self.speedy
            self.K_alpha = 1.8 #* self.speedy
            self.K_beta =  -0.3 #* self.speedy

        self.max_speed_m_per_sec = 0.05
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        # path tracking
        self.path_tracker = None
        if CONFIG_COURSE == INDOOR_COURSE:
            self.path = CONFIG_ROBOT_PATH
        else:
            self.path = STD_PATH
        self.path_current_pos = 0
        self.is_path_tracked = False
        self.path_tracking_pause_duration = 0

        # state that monitors difference between desired state, estimated state
        self.difference_state = E160_state()
        self.difference_state.set_state(0,0,0)

        # forward, rotational velocities
        self.v = 0.0
        self.w = 0.0
        self.was_forward = 0

        # Lab 4 Particle Filter
        # UPDATE THIS FOR ENCODER RESOLUTION
        self.PF = E160_PF(environment, 
                          self.width, 
                          self.wheel_radius, 
                          2 * math.pi * self.wheel_radius * CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10])

        # Final Project Unscented Kalman Filter
        INIT_TRANSLATION_VARIANCE = CONFIG_INIT_TRANSLATION_STDEV**2
        INIT_ANGLE_VARIANCE = CONFIG_INIT_ANGLE_STDEV**2
        self.var_ukf = np.zeros((3,3))
        self.var_ukf = np.zeros((3,3))
        self.var_ukf[0][0] = INIT_TRANSLATION_VARIANCE
        self.var_ukf[1][1] = INIT_TRANSLATION_VARIANCE
        self.var_ukf[2][2] = INIT_ANGLE_VARIANCE
        self.UKF = E160_UKF(environment,
                            np.array([self.state_ukf.x,
                                      self.state_ukf.y,
                                      self.state_ukf.theta]),
                            self.var_ukf, 
                            self.width, 
                            self.wheel_radius, 
                            2 * math.pi * self.wheel_radius * CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10])

        if CONFIG_FILTER == CONFIG_PARTICLE_FILTER:
            self.filter = self.PF
        elif CONFIG_FILTER == CONFIG_UNSCENTED_KALMAN_FILTER:
            self.filter = self.UKF

    def change_headers(self):
        self.make_headers()


    def update(self, deltaT):
        
        self.last_encoder_measurements[0] = self.encoder_measurements[0]
        self.last_encoder_measurements[1] = self.encoder_measurements[1]

        # get sensor measurements
        self.encoder_measurements, self.range_voltages = self.update_sensor_measurements(deltaT)
        if CONFIG_ROBOT_MODE == HARDWARE_MODE:
            self.range_measurements = []
            for i in range(len(self.range_voltages)-2):
                v = self.range_voltages[i]
                self.range_measurements.append(CONFIG_DISTANCE_CALIBRATION(max(v,1),i)/100)
            #self.range_measurements = [CONFIG_FORWARD_DISTANCE_CALIBRATION(max(v,1))/100 for v in self.range_voltages]
            self.range_measurements[0] = self.range_measurements[0] + CONFIG_FRONT_SENSOR_OFFSET_M
            self.range_measurements[1] = self.range_measurements[1] + CONFIG_LEFT_SENSOR_OFFSET_M 
            self.range_measurements[2] = self.range_measurements[2] + CONFIG_RIGHT_SENSOR_OFFSET_M 
        else:    
            self.range_measurements = self.range_voltages


        # update odometry
        delta_s, delta_theta = self.update_odometry(self.encoder_measurements)
        self.delta_state_odo = (delta_s, delta_theta)

        # localize
        self.state_odo = self.localize(self.state_odo, self.encoder_measurements, self.range_measurements)
        
        # localize with particle filter
        self.state_est = self.filter.LocalizeEst(self.encoder_measurements, self.last_encoder_measurements, self.range_measurements)
        # self.state_est = self.state_odo
        self.state_ctrl = self.state_est
        # self.state_est

        # to out put the true location for display purposes only. 
        self.state_draw = self.state_odo

        # call motion planner
        #self.motion_planner.update_plan()
        
        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)
    
    def update_sensor_measurements(self, deltaT):

        if CONFIG_IN_HARDWARE_MODE(self.environment.robot_mode):
            command = '$S @'
            self.environment.xbee.tx(dest_addr=self.address, data=command)
            
            update = self.environment.xbee.wait_read_frame()
            
            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            # left encoder, right encoder
            encoder_measurements = [encoder_measurements[1], encoder_measurements[0]]
            range_measurements = data[:-2]
            
        
        # obtain sensor measurements
        elif CONFIG_IN_SIMULATION_MODE(self.environment.robot_mode):
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            added_sensor_noise = [random.gauss(0,CONFIG_SENSOR_NOISE_SIM),
                                  random.gauss(0,CONFIG_SENSOR_NOISE_SIM),
                                  random.gauss(0,CONFIG_SENSOR_NOISE_SIM)]
            sensor1 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[0]) + added_sensor_noise[0]
            sensor2 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[1]) + added_sensor_noise[1]
            sensor3 = self.simulate_range_finder(self.state_odo, self.PF.sensor_orientation[2]) + added_sensor_noise[2]
            range_measurements = [sensor2, sensor3, sensor1]
        
        return encoder_measurements, range_measurements
        
        
    def localize(self, state_est, encoder_measurements, range_measurements):
        delta_s, delta_theta = self.delta_state_odo
        state_est = self.update_state(state_est, delta_s, delta_theta)
    
        return state_est
    
    
    def update_control(self, range_measurements):

        old_L = self.L
        old_R = self.R
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            R = self.manual_control_right_motor
            L = self.manual_control_left_motor
            L, R = self.rampSpeed(L, R, old_L, old_R)
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":

            L = self.testing_power_L
            R = self.testing_power_R
            # R = self.R_motor_scaling_factor*self.testing_power_L

            L, R = self.rampSpeed(L, R, old_L, old_R)
            # power = 0

            if CONFIG_LAB_NUMBER == 1:
                power = self.lab1_controller(range_measurements)
                R = L = power

            elif CONFIG_LAB_NUMBER == 2:
                # write lab2 controller
                pass

            elif CONFIG_LAB_NUMBER == 3:
                R, L = self.lab3_controller(range_measurements)

            elif CONFIG_LAB_NUMBER == 4:
                R, L = self.lab4_controller(range_measurements)

            elif CONFIG_LAB_NUMBER == 5:
                pass

            elif CONFIG_LAB_NUMBER == 6:
                R, L = self.lab4_controller(range_measurements)

            else:
                print('Inappropriate lab number specified. Please enter a number from 1-5.')

        return R, L
        

    def send_control(self, R, L, deltaT):
        
        # send to actual robot !!!!!!!!
        if self.environment.robot_mode == "HARDWARE MODE":
            if (L < 0):
                LDIR = 0
            else:
                LDIR = 1

            if (R < 0):
                RDIR = 0
            else:
                RDIR = 1
            # PWM is positive 8 bit number, can change this to be floats with PI controller code.
            RPWM = (2*RDIR - 1)*int(abs(R))
            LPWM = (2*LDIR - 1)*int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            # command should have the desired tick rate per 20 ms
            # 256 -> 1400 per 100 ms = 280 per 20 ms
            right_tick_rate =  RPWM
            left_tick_rate =  LPWM
            command = '$T ' + str(right_tick_rate) + ' ' + str(left_tick_rate)  + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
        

    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = -int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        return [left_encoder_measurement, right_encoder_measurement]
    
    def simulate_range_finder(self, state, sensorT):
        '''Simulate range readings, given a simulated ground truth state'''
        p = self.PF.Particle(state.x, state.y, state.theta, 0)

        return self.PF.FindMinWallDistance(p, self.environment.walls, sensorT)

    # clean up this function
    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

        # ****************** Additional Student Code: Start ************

        left_encoder_measurement = encoder_measurements[0]
        right_encoder_measurement = encoder_measurements[1]  
        last_left_encoder_measurement = self.last_encoder_measurements[0]
        last_right_encoder_measurement = self.last_encoder_measurements[1]
        self.delta_left = float(left_encoder_measurement - last_left_encoder_measurement)
        self.delta_right = float(right_encoder_measurement - last_right_encoder_measurement)    

        if self.is_first_run:
            self.delta_right = 0
            self.delta_left = 0
            self.is_first_run = False

        # cause the lab said so I like my name better
        diffEncoder0 = self.delta_left
        diffEncoder1 = self.delta_right

        wheel_circumference = 2 * math.pi * self.wheel_radius


        # TODO: implement calibration from ticks to centimeters
        # left_distance = (self.delta_left / self.encoder_resolution) * wheel_circumference
        # right_distance = (self.delta_right / self.encoder_resolution) * wheel_circumference
        left_distance  = self.delta_left  * CONFIG_CM_TO_M *  CONFIG_LEFT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10]
        right_distance = self.delta_right * CONFIG_CM_TO_M * CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10]

        delta_s = (left_distance + right_distance) / 2
        delta_theta = (right_distance - left_distance) / (2 * self.radius)


        # set current measurements as the last for next cycle
        # self.last_encoder_measurements[0] = left_encoder_measurement
        # self.last_encoder_measurements[1] = right_encoder_measurement

        # ****************** Additional Student Code: End ************
            
        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta 


    def update_state(self, state, delta_s, delta_theta):
        
        # ****************** Additional Student Code: Start ************

        x_new = state.x + math.cos(state.theta + delta_theta / 2) * delta_s
        y_new = state.y + math.sin(state.theta + delta_theta / 2) * delta_s

        theta_new = self.normalize_angle(state.theta + delta_theta)
        state.add_theta(delta_theta)
        state.set_state(x_new, y_new, theta_new)
        
        # ****************** Additional Student Code: End ************
            
        # keep this to return the updated state
        return state
        
        
        
    ######## DATA LOGGING ########  
    def make_headers(self):
        f = open(self.file_name, 'a+')
        # f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        names = ['desired_tick_rate_L', 'desired_tick_rate_R', 'state_odo_x', 'state_odo_y', 'state_odo_theta', 'deltaleftEnc', 'deltarightEnc', 'pf_state_x', 'pf_state_y', 'pf_state_theta', 'est_error_x','est_error_y','est_error_theta','variance']
        f.write(' '.join(names) + '\n')
        f.close()

        
    def log_data(self):
        f = open(self.file_name, 'a+')

        # log distance from wall to 2 decimal places
        # alpha = round(self.R_motor_scaling_factor, 4)
        if hasattr(self.filter, "variance"):
            log_var = self.filter.variance 
        else:
            log_var =  ""
        error = [self.state_odo.x-self.state_est.x, 
                    self.state_odo.y-self.state_est.y, 
                    self.state_odo.theta-self.state_est.theta]
        data = [str(self.L), str(self.R), str(self.state_odo),
                 str(self.delta_left), str(self.delta_right), str(self.state_est),
                 str(error), str(log_var)]
        
        f.write(' '.join(data) + '\n')
        f.close()
    ###### END DATA LOGGING ######    
        
    def set_manual_control_motors(self, R, L):
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)  


    ###### LAB CONTROLLERS ######
    def lab1_controller(self, range_measurements):
        # Lab 1 control code
        if range_measurements[0] > 0:
            forward_distance = range_measurements[0]
            distance_cm = CONFIG_FORWARD_DISTANCE_CALIBRATION(forward_distance)
            # For lab 1, update y-state to be distance from wall
            self.state_odo.set_state(0, distance_cm, 0)
        else:
            # if voltage is 0, don't move
            distance_cm = CONFIG_ERROR_DISTANCE_CM
            power = 0
        
        error_cm = (distance_cm - CONFIG_DESIRED_DISTANCE_CM)

        if abs(error_cm) < CONFIG_ERROR_THRESHOLD_CM:
            power = 1
        else:
            power = CONFIG_PROPORTIONAL_CONSTANT*(error_cm/CONFIG_DESIRED_DISTANCE_CM)
            
            # sign is +1 if move forwards, -1 if backwards
            if power != 0:
                sign = power/abs(power)
            else:
                sign = 1

            # use max/min speeds
            power = sign * min(power, CONFIG_MAX_POWER)
            power = sign * max(power, CONFIG_MIN_POWER)

             
    def lab2_controller(self, range_measurements):
        pass

    # LAB 3
    def lab3_controller(self, range_measurements):
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            self.path_tracker = self.create_path_tracker(self.path)

            desiredWheelSpeedR, desiredWheelSpeedL = (0, 0)

            if self.is_path_tracked:
                self.path_current_pos = 0
            else:
                desiredWheelSpeedR, desiredWheelSpeedL = next(self.path_tracker)

        return desiredWheelSpeedR, desiredWheelSpeedL
    

    def lab4_controller(self, range_measurements):
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":
            self.path_tracker = self.create_path_tracker(self.path)

            desiredWheelSpeedR, desiredWheelSpeedL = (0, 0)

            if self.is_path_tracked:
                self.path_current_pos = 0
            else:
                desiredWheelSpeedR, desiredWheelSpeedL = next(self.path_tracker)

        return desiredWheelSpeedR, desiredWheelSpeedL

    ###### END LAB CONTROLLERS ######

    ###### LAB 3 HELPER FUNCTIONS ######
    def point_tracker_control(self):

        right_ticks_per_sec = 0
        left_ticks_per_sec  = 0

        # set acceptable thresholds for point tracking
        acceptable_distance = CONFIG_DISTANCE_THRESHOLD_M 
        acceptable_angle = CONFIG_ANGLE_THRESHOLD_RAD

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:

            # 1. Calculate changes in x, y.
            self.difference_state = self.state_ctrl.get_state_difference(self.state_des)
            Dx = self.difference_state.x
            Dy = self.difference_state.y
            Dtheta = self.difference_state.theta

            # decide whether or not to use forward/reverse controller
            go_forward = self._toggle_forward_reverse()

            distance_to_point = math.sqrt(Dx**2 + Dy**2)
            ideal_heading_to_point = math.atan2(go_forward * Dy, go_forward * Dx)

            reached_goal_state, \
            at_point, \
            care_about_trajectory, \
            beta_modifier = self._update_modifiers(acceptable_distance, 
                                                   acceptable_angle)
            if reached_goal_state:
                self.point_tracked = True
                self.was_forward = 0
                return (0, 0)

            # 2. Calculate position of \rho, \alpha, \beta, respectively
            distance_to_point = distance_to_point
            angle_error = self.short_angle(care_about_trajectory * (-self.state_odo.theta
                                                   + ideal_heading_to_point))
            negated_angle_final = ((1-at_point) * angle_error 
                                   - self.state_odo.theta
                                   + self.state_des.theta)

            # currently unused, may use in future
            alpha_modifier = 1
            # if abs(angle_error) < math.pi/8:
            #     alpha_modifier = 0.1

            # RENAME THIS VARIABLE
            if at_point == 1:
                negated_angle_final = self.short_angle(negated_angle_final)

            # 3. Identify desired velocities (bound by max velocity)
            self.v = care_about_trajectory * go_forward * self.K_rho * distance_to_point
            self.w = (alpha_modifier * self.K_alpha * angle_error 
                     + beta_modifier * self.K_beta  * negated_angle_final)

            # 4a. Determine desired wheel rotational velocities using desired robot velocities
            # Assuming CW is positive, then left wheel positively correlated w/ velocity
            left_rad_per_sec = 0.5 * (self.w + (self.v/self.radius))
            right_rad_per_sec = -0.5 * (self.w - (self.v/self.radius))

            # 4b. Convert rotational velocities to wheel velocities in cm/s.
            wheel_rotational_vel_to_m_per_sec = 2*self.radius/self.wheel_radius
            right_cm_per_sec = (right_rad_per_sec 
                                * wheel_rotational_vel_to_m_per_sec
                                * CONFIG_M_TO_CM)
            left_cm_per_sec = (left_rad_per_sec 
                                * wheel_rotational_vel_to_m_per_sec
                                * CONFIG_M_TO_CM)

            # 4c. max speed of 5 cm/s reduce other angle to allow same ratio
            max_wheel_velocity = max(abs(right_cm_per_sec), abs(left_cm_per_sec))
            if max_wheel_velocity > 5:
                right_cm_per_sec *= (5/max_wheel_velocity)
                left_cm_per_sec *= (5/max_wheel_velocity)
                
            # 4d. Convert wheel velocities in cm/s to wheel velocities in ticks/s.
            # TODO: Finish up map, design interpolation for cm to ticks conversion
            if CONFIG_IN_HARDWARE_MODE(self.environment.robot_mode):
                right_ticks_per_sec = (left_cm_per_sec 
                                       / CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10])
                left_ticks_per_sec  = (right_cm_per_sec 
                                       / CONFIG_LEFT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10])
            if CONFIG_IN_SIMULATION_MODE(self.environment.robot_mode):
                right_ticks_per_sec = (right_cm_per_sec 
                                       / CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10])
                left_ticks_per_sec  = (left_cm_per_sec 
                                       / CONFIG_LEFT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10])


            # 5. Check if we are close enough to desired destination
            # TODO: add a threshold
            self.point_tracked = False

        else:
            # the desired point has been tracked, so don't move
            pass
                
        return right_ticks_per_sec, left_ticks_per_sec


    def _toggle_forward_reverse(self):
        """
        Selects, for point tracking, whether a robot should move forward (1) or
        in reverse (-1) to get to the destination.
        """
        # going forward from start if change in theta in [-pi/2, pi/2]
        go_forward = 1

        # Get translational differences between current and desired states
        Dx = self.difference_state.x
        Dy = self.difference_state.y

        # Identify the heading from the robot to the destination (x_des, y_des)
        # Note: Only considers translation, ignores current angle, dest angle
        ideal_heading = self.normalize_angle(math.atan2(Dy, Dx))

        # Get estimated angle of robot w.r.t. global frame
        robot_heading = self.state_odo.theta

        # Identify difference between taking direct path and current robot heading
        absolute_heading_difference = abs(self.normalize_angle(ideal_heading - robot_heading))

        # if robot was previously unmoving, 
        # then select forward if the heading difference is in [-pi/2, pi/2]
        #      i.e. go backward if difference in [-pi, -pi/2) or (pi/2, pi]
        # otherwise robot was previously moving,
        # then select same direction if the heading difference 
        #      is in [-pi/2-bias_for_same_dir, pi/2+bias_for_same_dir]

        threshold = CONFIG_QUARTER_TURN + self.was_forward * CONFIG_POINT_TRACKING_ANGLE_BIAS
        is_facing_wrong_direction = absolute_heading_difference > threshold
        go_forward = -1 if is_facing_wrong_direction else 1

        self.was_forward = go_forward
        return go_forward

    def _update_modifiers(self, acceptable_distance, acceptable_angle):
        """
        Updates control modifiers.
        """
        self.difference_state = self.state_odo.get_state_difference(self.state_des)
        Dx = self.difference_state.x
        Dy = self.difference_state.y
        Dtheta = self.difference_state.theta

        distance_to_point = math.sqrt(Dx**2 + Dy**2)

        # initialize modifier variables
        reached_destination = False
        at_point = 0
        care_about_trajectory = 1
        beta_modifier = 0.1

        if(distance_to_point < acceptable_distance and abs(Dtheta) < acceptable_angle):
            reached_destination = True

        if(distance_to_point < 2*acceptable_distance):
            at_point = 1

        if(distance_to_point < acceptable_distance/2):
            care_about_trajectory = 0
            # can switch K_beta to be positive after translation is over
            beta_modifier *= -1.0 #/self.speedy

        return reached_destination, at_point, care_about_trajectory, beta_modifier

    ###### END LAB 3 HELPER FUNCTIONS ######

    def create_path_tracker(self, path):
        """
        A generator that, given a list of waypoints (E160_state objects), uses
        ``point_tracker_control`` to follow the path.
        """
        self.point_tracked = False
        self.is_path_tracked = False
        right_power, left_power = (0, 0)

        while not self.is_path_tracked:
            point = path[self.path_current_pos]
            point.theta = self.short_angle(point.theta)
            self.state_des = point

            # track the point for one iteration
            right_power, left_power = self.point_tracker_control()

            # if the previous point has been reached, go to the next point
            if self.point_tracked:
                print("Point ", self.path_current_pos, ", ", point, ", is tracked.")
                self.path_current_pos += 1
                # slight delay for testing reasons
                time.sleep(self.path_tracking_pause_duration)
            self.point_tracked = False

            # not all checkpoints reached
            self.is_path_tracked = not(self.path_current_pos < len(path))

            yield right_power, left_power

        self.is_path_tracked = True
        yield right_power, left_power


     ###### END LAB 3 HELPER FUNCTIONS ######

    def normalize_angle(self, theta):
        '''
        Returns an angle in range (-pi, pi]
        '''
        out_angle = theta % (2 * math.pi)
        if out_angle > math.pi:
            out_angle = out_angle - 2 * math.pi
        return out_angle
    
    def rampSpeed(self, L ,R, old_L, old_R):
        delta_L = L-old_L
        delta_R = R-old_R
        ramped_L = L
        ramped_R = R

        if abs(delta_L) > 0 and L != 0 and R != 0:
            ramped_delta_L = max(min(delta_L,CONFIG_RAMP_CONSTANT),-CONFIG_RAMP_CONSTANT)
            ramped_L = old_L + ramped_delta_L
        if abs(delta_R) > 0 and L != 0 and R != 0:
            ramped_delta_R = max(min(delta_R,CONFIG_RAMP_CONSTANT),-CONFIG_RAMP_CONSTANT)
            ramped_R = old_R + ramped_delta_R

        return ramped_L, ramped_R

    def set_manual_control_motors(self, R, L):
        
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)

       #concern could mess up the final orientaton
    
    def short_angle(self,theta):
        if theta > math.pi:
            theta = theta - 2 * math.pi
        if theta < -math.pi:
            theta = 2*math.pi + theta
        return theta

