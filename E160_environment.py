from E160_config import *
from E160_robot import *
from E160_state import *
from E160_wall import *
import serial
import time
from xbee import XBee

class E160_environment:

    
    def __init__(self):
        self.width = 2.0
        self.height = 2.0 # was 2.0 in old code, change for lab 3
        in2m = 0.0254 
        # set up walls, putting top left point first
        self.walls = []
        self.walls.append(E160_wall([x*in2m for x in [-19, 8.5, -19, -10.5]],"vertical"))
        self.walls.append(E160_wall([x*in2m for x in [-19, 8.5, -22, 8.5]],"horizontal"))
        self.walls.append(E160_wall([x*in2m for x in [-22, 22.5, -22, 8.5]],"vertical"))
        self.walls.append(E160_wall([x*in2m for x in [-3, 50.5, 7, 50.5]],"horizontal"))
        self.walls.append(E160_wall([x*in2m for x in [19.5, 7.5, 19.5, -10.5]],"vertical"))
        self.walls.append(E160_wall([x*in2m for x in [-19, -10.5, 19.5, -10.5]],"horizontal"))

        self.walls.append(E160_wall([x*in2m for x in [19.5, 7.5, 25.5, 23.5]],"horizontal"))
            
        # create vars for hardware vs simulation
        self.robot_mode = CONFIG_ROBOT_MODE
        self.control_mode = CONFIG_CONTROL_MODE

        # setup xbee communication
        if (self.robot_mode == "HARDWARE MODE"):
            self.serial_port = serial.Serial(CONFIG_PORT, 9600)
            print("Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")
        
        # Setup the robots
        self.num_robots = 1
        self.robots = []
        for i in range (0,self.num_robots):
            
            # TODO: assign different address to each bot
            r = E160_robot(self, '\x00\x0C', i)
            self.robots.append(r)
    
    def update_robots(self, deltaT):
        
        # loop over all robots and update their state
        for r in self.robots:
            
            # set the control actuation
            r.update(deltaT)
        
        
    def log_data(self):
        
        # loop over all robots and update their state
        for r in self.robots:
            r.log_data()
            
    def quit(self):
        self.xbee.halt()
        self.serial.close()
            
            
            