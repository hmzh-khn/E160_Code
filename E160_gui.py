"""
E160_gui.py
"""
import sys
from E160_config import CONFIG_DELTA_T
import time
from E160_environment import *
from E160_graphics import *

def main():  
       
    # instantiate robot navigation classes
    environment = E160_environment()
    graphics = E160_graphics(environment)
    
    # set time step size in seconds
    deltaT = CONFIG_DELTA_T
    # loop over time
    while True:
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break
        
        # update robots
        environment.update_robots(deltaT)
        
        # log all the robot data
        environment.log_data()
    
        # maintain timing
        time.sleep(deltaT)
   
if __name__ == "__main__":
    main()
