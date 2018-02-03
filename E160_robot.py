
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
        self.wheel_radius = 0.03
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
        state_est = self.update_state(state_est, delta_s, delta_theta)
    
        return state_est
    
    
    def update_control(self, range_measurements):
        
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            R = self.manual_control_right_motor
            L = self.manual_control_left_motor
            
        elif self.environment.control_mode == "AUTONOMOUS CONTROL MODE":        

            # Lab 1 control code
            if range_measurements[0] > 0:
                forward_distance = range_measurements[0]
                distance_cm = CONFIG_FORWARD_DISTANCE_CALIBRATION(forward_distance)
                # For lab 1, update y-state to be distance from wall
                self.state_est.set_state(0,distance_cm,0)
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

            R = L = power

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
            RPWM = int(abs(R))
            LPWM = int(abs(L))

            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            self.environment.xbee.tx(dest_addr = self.address, data = command)
            

    def simulate_encoders(self, R, L, deltaT):
        gain = 10
        right_encoder_measurement = -int(R*gain*deltaT) + self.last_simulated_encoder_R
        left_encoder_measurement = -int(L*gain*deltaT) + self.last_simulated_encoder_L
        self.last_simulated_encoder_R = right_encoder_measurement
        self.last_simulated_encoder_L = left_encoder_measurement
        
        print("simulate_encoders", R, L, right_encoder_measurement, left_encoder_measurement)
        return [left_encoder_measurement, right_encoder_measurement]
    
        
    def update_odometry(self, encoder_measurements):

        delta_s = 0
        delta_theta = 0

        # ****************** Additional Student Code: Start ************

        
  

        # ****************** Additional Student Code: End ************
            
        # keep this to return appropriate changes in distance, angle
        return delta_s, delta_theta 

    
    
    
    def update_state(self, state, delta_s, delta_theta):
        
        # ****************** Additional Student Code: Start ************
                     
        
        # ****************** Additional Student Code: End ************
            
        # keep this to return the updated state
        return state
        
        
        
        
    def make_headers(self):
        f = open(self.file_name, 'a+')
        # f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.write('distance from wall\n')
        f.close()

        
        
    def log_data(self):
        f = open(self.file_name, 'a+')
        
        # log distance from wall to 2 decimal places
        dist_from_wall_cm = round(self.state_est.y, 2)
        data = [str(dist_from_wall_cm)]
        
        f.write(' '.join(data) + '\n')
        f.close()
        
        
    def set_manual_control_motors(self, R, L):
        
        self.manual_control_right_motor = int(R*256/100)
        self.manual_control_left_motor = int(L*256/100)                                                         
   

