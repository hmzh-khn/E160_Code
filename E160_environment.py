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
        m2in = 39.37
        # set up walls, putting top left point first
        self.walls = []
        
        if(CONFIG_COURSE == INDOOR_COURSE):
        # something
            indoor_points = [(0,0),
            (22,0), 
            (28.75, 0),
            (22,-17),
            (28.75, -17),
            (22, -27.25),
            (25.25, -27.25),
            (0, -44.25),
            (0, -49.5),
            (14.375, -44.25),
            (14.375, -49.5),
            (0,-72),
            (70,-72),
            (64.75 ,-58.75),
            (53, 0),
            (53,0.25),
            (77, 0.25),
            (77, -55.25),
            (36,-43),
            (48.5,-32.5),
            (56.5,-40),
            (44.5, -50.5),
            (25.25,-17)]
            indoor_walls = [(0,14),(0,11),(11,12),(12,13),(13,17),(17,16),(16,15),(15,14),(1,5),(5,6),(4,2),(6,22),(22,4),(4,22),(7,9),(9,10),(10,8),(19,20),(20,21),(21,18),(18,19)]
            for wall in indoor_walls:
                point1 = indoor_points[wall[0]]
                point2 = indoor_points[wall[1]]
                orientStr = "horizontal"
                if point1[0] > point2[0]:
                    #we good
                    pass
                elif point1[0] == point2[0]:
                    orientStr = "vertical"
                    if point1[1] < point2[1]:
                        temp = point1
                        point1 = point2
                        point2 = temp
                else:
                    temp = point1
                    point1 = point2
                    point2 = temp
                x1 = (point1[0] - m2in/2) * in2m
                y1 = (point1[1] + m2in/2) * in2m
                x2 = (point2[0] - m2in/2) * in2m
                y2 = (point2[1] + m2in/2) * in2m
                self.walls.append(E160_wall([x1,y1,x2,y2],orientStr))
        else:
            self.walls.append(E160_wall([x*in2m for x in [-19, 8.5, -19, -10.5]],"vertical"))
            self.walls.append(E160_wall([x*in2m for x in [-19, 8.5, -22, 8.5]],"horizontal"))
            self.walls.append(E160_wall([x*in2m for x in [-22, 22.5, -22, 8.5]],"vertical"))
            self.walls.append(E160_wall([x*in2m for x in [-3, 50.5, 7, 50.5]],"horizontal"))
            self.walls.append(E160_wall([x*in2m for x in [19.5, 7.5, 19.5, -10.5]],"vertical"))
            self.walls.append(E160_wall([x*in2m for x in [-19, -10.5, 19.5, -10.5]],"horizontal"))
            self.walls.append(E160_wall([x*in2m for x in [19.5, 7.5, 25.5, 23.5]],"horizontal"))
            self.walls.append(E160_wall([x*in2m for x in [-19.0, 23.5, 25.5, 23.5]],"horizontal"))
        

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
            
            
            