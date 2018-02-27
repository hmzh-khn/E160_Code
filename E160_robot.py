
from E160_config import *
from E160_state import *
import math
import datetime


class E160_robot:

    def __init__(self, environment, address, robot_id):
        self.environment = environment
        self.state_est = E160_state()
        self.state_est.set_state(0,0,0)
        self.state_des = E160_state()
        self.state_des.set_state(0,0,0)

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
        self.delta_state = (0, 0)
        self.testing_power_L = 0
        self.testing_power_R = 0

        # Lab 3
        self.K_rho = 0.005#1.0
        self.K_alpha = 0.02#2.0
        self.K_beta = -0.0005#-0.5
        self.max_speed_m_per_sec = 0.05
        self.point_tracked = True
        self.encoder_per_sec_to_rad_per_sec = 10

        # state that monitors difference between desired state, estimated state
        self.difference_state = E160_state()
        self.difference_state.set_state(0,0,0)

        # forward, rotational velocities
        self.v = 0.0
        self.w = 0.0

    def change_headers(self):
        self.make_headers()


    def update(self, deltaT):
        
        # get sensor measurements
        self.encoder_measurements, self.range_measurements = self.update_sensor_measurements(deltaT)

        # localize
        self.state_est = self.localize(self.state_est, self.encoder_measurements, self.range_measurements)
        
        # call motion planner
        #self.motion_planner.update_plan()
        
        # determine new control signals
        self.R, self.L = self.update_control(self.range_measurements)
        
        # send the control measurements to the robot
        self.send_control(self.R, self.L, deltaT)
    
 
    
    def update_sensor_measurements(self, deltaT):
        
        if self.environment.robot_mode == "HARDWARE MODE":
            command = '$S @'
            self.environment.xbee.tx(dest_addr=self.address, data=command)
            
            update = self.environment.xbee.wait_read_frame()
            
            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]
        
        # obtain sensor measurements
        elif self.environment.robot_mode == "SIMULATION MODE":
            encoder_measurements = self.simulate_encoders(self.R, self.L, deltaT)
            range_measurements = [0,0,0]
        
        return encoder_measurements, range_measurements
        
        
    def localize(self, state_est, encoder_measurements, range_measurements):
        delta_s, delta_theta = self.update_odometry(encoder_measurements)
        self.delta_state = (delta_s, delta_theta)
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
                pass
            else:
                # do nothing
                pass

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
            right_tick_rate = RPWM
            left_tick_rate = LPWM
            command = '$T ' + str(right_tick_rate) + ' ' + str(left_tick_rate)  + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
        

    def simulate_encoders(self, R, L, deltaT):
        right_encoder_measurement = -int(R*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*self.encoder_per_sec_to_rad_per_sec*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        return [left_encoder_measurement, right_encoder_measurement]
    

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
        self.last_encoder_measurements[0] = left_encoder_measurement
        self.last_encoder_measurements[1] = right_encoder_measurement

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
        
        
        
        
    def make_headers(self):
        f = open(self.file_name, 'a+')
        # f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        names = ['desired_tick_rate_L', 'desired_tick_rate_R', 'state_est_x', 'state_est_y', 'state_est_theta', 'deltaleftEnc', 'deltarightEnc']
        f.write(' '.join(names) + '\n')
        f.close()

        
        
    def log_data(self):
        f = open(self.file_name, 'a+')

        # log distance from wall to 2 decimal places
        # alpha = round(self.R_motor_scaling_factor, 4)
        data = [str(self.L), str(self.R), str(self.state_est), str(self.delta_left), str(self.delta_right)]
        
        f.write(' '.join(data) + '\n')
        f.close()
        
        
    def set_manual_control_motors(self, R, L):
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)  


    def lab1_controller(self, range_measurements):
        # Lab 1 control code
        if range_measurements[0] > 0:
            forward_distance = range_measurements[0]
            distance_cm = CONFIG_FORWARD_DISTANCE_CALIBRATION(forward_distance)
            # For lab 1, update y-state to be distance from wall
            self.state_est.set_state(0, distance_cm, 0)
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

    ############ LAB 3 #################
    def lab3_controller(self, range_measurements):
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            desiredWheelSpeedR = self.manual_control_right_motor
            desiredWheelSpeedL = self.manual_control_left_motor
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":   
            desiredWheelSpeedR, desiredWheelSpeedL = self.point_tracker_control()
            
        return desiredWheelSpeedR, desiredWheelSpeedL


    def point_tracker_control(self):

        wheel_velocity_right_ticks_per_sec = 0
        wheel_velocity_left_ticks_per_sec  = 0

        # If the desired point is not tracked yet, then track it
        if not self.point_tracked:
            ############ Student code goes here ############################################
            
            # 1. Calculate changes in x, y.
            self.difference_state = self.state_est.get_state_difference(self.state_des)
            Dx = self.difference_state.x
            Dy = self.difference_state.y
            Dtheta = self.difference_state.theta

            at_point = 1
            care_about_tracjectory = 1

            acceptable_distance = 0.01
            acceptable_angle = acceptable_distance
            distance = math.sqrt(Dx**2 + Dy**2)

            if(distance < acceptable_distance and abs(Dtheta) < acceptable_angle):
                self.point_tracked = True
                return (0, 0)

            if(distance < acceptable_distance):
                at_point = 1
                care_about_tracjectory = max((distance - acceptable_distance/2) / acceptable_distance , 0)
                print('switch to reduced distance')

            if(distance< acceptable_distance/2):
                at_point = 1
                care_about_tracjectory = 0
                print('switch to rotate control')


            # going forward if change in theta in [-pi/2, pi/2]
            is_forward = 1
            # going backward if in [-pi, -pi/2) or (pi/2, pi]
            if abs(math.atan2(Dy, Dx)) > math.pi/2:
                is_forward = -1

            # 2. Calculate position of \rho, \alpha, \beta, respectively
            distance_to_point = math.sqrt(Dx**2 + Dy**2)
            angle_error = (-self.state_est.theta + math.atan2(is_forward * Dy, 
                                                             is_forward * Dx)) * care_about_tracjectory
            negated_angle_final = -self.state_est.theta - angle_error + self.state_des.theta
            #negated_angle_final = self.short_angle(negated_angle_final)
            # 3. Identify desired velocities (bound by max velocity)
            # TODO: THINK ABOUT HOW TO DEAL WITH LIMIT CYCLE
            self.v = is_forward * self.K_rho * distance_to_point

            self.w = self.K_alpha * angle_error + self.K_beta * negated_angle_final
            print('Bearing: ',self.state_est.theta,' From Final: ',Dtheta, "NAF: ", negated_angle_final)
            
            print('w: ',self.w,'v: ',self.v)
            # 4a. Determine desired wheel rotational velocities using desired robot velocities
            # Assuming CW is positive, then left wheel positively correlated w/ velocity
            wheel_rotational_velocity_left_rad_per_sec = 0.5 * (self.w + (self.v/self.radius))
            wheel_rotational_velocity_right_rad_per_sec = -0.5 * (self.w - (self.v/self.radius))

            # 4b. Convert rotational velocities to wheel velocities in cm/s.
            robot_rotational_vel_to_wheel_rotational_vel_m_per_sec = 2*self.radius/self.wheel_radius
            wheel_velocity_right_cm_per_sec = (wheel_rotational_velocity_right_rad_per_sec 
                             * robot_rotational_vel_to_wheel_rotational_vel_m_per_sec
                             * CONFIG_M_TO_CM)
            wheel_velocity_left_cm_per_sec = (wheel_rotational_velocity_left_rad_per_sec 
                             * robot_rotational_vel_to_wheel_rotational_vel_m_per_sec
                             * CONFIG_M_TO_CM)

            # 4c. max speed of 5 cm/s reduce other angle to allow same ratio
            max_wheel_velocity = max(abs(wheel_velocity_right_cm_per_sec), abs(wheel_velocity_left_cm_per_sec))
            if max_wheel_velocity > 5:
                wheel_velocity_right_cm_per_sec *= (5/max_wheel_velocity)
                wheel_velocity_left_cm_per_sec *= (5/max_wheel_velocity)
            # 4d. Convert wheel velocities in cm/s to wheel velocities in ticks/s.
            # TODO: Finish up map, design interpolation for cm to ticks conversion
            wheel_velocity_right_ticks_per_sec = wheel_velocity_right_cm_per_sec / CONFIG_RIGHT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10]
            wheel_velocity_left_ticks_per_sec  = wheel_velocity_left_cm_per_sec / CONFIG_LEFT_CM_PER_SEC_TO_TICKS_PER_SEC_MAP[10]

            print(wheel_velocity_right_cm_per_sec, wheel_velocity_left_cm_per_sec)

            # 5. Check if we are close enough to desired destination
            # TODO: add a threshold
            self.point_tracked = False


            
        # the desired point has been tracked, so don't move
        else:
            pass
                
        return wheel_velocity_right_ticks_per_sec, wheel_velocity_left_ticks_per_sec

    ############ END LAB 3 #################

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
        print("definitely not working yet")
        return theta

