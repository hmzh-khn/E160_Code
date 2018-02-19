
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
        #self.v = 0.05
        #self.w = 0.1
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
        self.encoder_resolution = 1440
        
        self.last_encoder_measurements = [0,0]
        self.encoder_measurements = [0,0]
        self.range_measurements = [0,0,0]
        self.last_simulated_encoder_R = 0
        self.last_simulated_encoder_L = 0

        self.delta_right = 0
        self.delta_left = 0

        self.first_run_flag = 1

        # stores changes in deltaS, deltaTheta
        self.delta_state = (0, 0)
        # self.R_motor_scaling_factor = CONFIG_R_MOTOR_SCALING_FACTOR
        self.testing_power_L = 0
        self.testing_power_R = 0

        # self.sum

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
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            
            update = self.environment.xbee.wait_read_frame()
            
            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = data[-2:]
            range_measurements = data[:-2]
        
        # obtain sensor measurements !!!!!! Chris
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

            # print("specified power (L,R) - ", (L,R))
            L, R = self.rampSpeed(L, R, old_L, old_R)
            # power = 0

            if CONFIG_LAB_NUMBER == 1:
                power = self.lab1_controller(range_measurements)
                R = L = power

            elif CONFIG_LAB_NUMBER == 2:
                # write lab2 controller
                pass
            else:
                # do nothing
                pass

        # print("power (L,R) - ", (L,R), (old_L,old_R))
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
            RPWM = 14*(2*RDIR - 1)*int(abs(R))
            LPWM = 14*(2*LDIR - 1)*int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            # command should have the desired tick rate per 20 ms
            # 256 -> 1400 per 100 ms = 280 per 20 ms
            right_tick_rate = RPWM
            left_tick_rate = LPWM
            command = '$T ' + str(right_tick_rate) + ' ' + str(left_tick_rate)  + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            

    def simulate_encoders(self, R, L, deltaT):
        gain = 10
        right_encoder_measurement = -int(R*gain*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*gain*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        return [left_encoder_measurement, right_encoder_measurement]
    
        
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

        if self.first_run_flag:
            self.delta_right = 0
            self.delta_left = 0
            self.first_run_flag = 0

        #cause the lab said so I like my name better
        diffEncoder0 = self.delta_left
        diffEncoder1 = self.delta_right

        wheel_circumference = 2 * math.pi * self.wheel_radius


        # TODO: implement calibration from ticks to centimeters
        # left_distance = (self.delta_left / self.encoder_resolution) * wheel_circumference
        # right_distance = (self.delta_right / self.encoder_resolution) * wheel_circumference
        left_distance  = self.delta_left  * CONFIG_CM_TO_M *  CONFIG_LEFT_CM_TO_TICKS_MAP[100]
        right_distance = self.delta_right * CONFIG_CM_TO_M * CONFIG_RIGHT_CM_TO_TICKS_MAP[100]

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

    def normalize_angle(self, theta):
        '''makes the angle normal but not normal (pi/2)'''
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


        # print("ramped power (L,R) - ", (ramped_L,ramped_R)) 
        return ramped_L, ramped_R
