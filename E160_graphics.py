from E160_config import *
import math
import sys
# Use correct package given version of python
if sys.version_info[0] < 3:
    from Tkinter import *
else:
    from tkinter import *
from E160_robot import *
from E160_robot import *
from PIL import Image, ImageTk


class E160_graphics:
    
    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        self.top_frame = Frame(self.tk)
        self.top_frame.pack(anchor = N)
        self.north_east_frame = Frame(self.top_frame)
        self.north_east_frame.pack(anchor = NE)
        self.north_west_frame = Frame(self.top_frame)
        self.north_west_frame.pack(anchor = NW)
        self.bottom_frame = Frame(self.tk)
        self.bottom_frame.pack(side = BOTTOM)

        self.typing_frame = Frame(self.bottom_frame)
        self.typing_frame.pack(side = TOP)

        self.scale = CONFIG_WINDOW_SCALE
        self.canvas = Canvas(self.tk, width=self.environment.width*self.scale, height=self.scale* self.environment.height)
        self.tk.title("E160 - Autonomous Robot Navigation")
        self.canvas.bind("<Button-1>", self.callback)
        self.canvas.pack()
        self.gui_stopped = False
        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.R = 0
        self.L = 0
        
        # attempt at typing input
        power_text=StringVar()
        power_text.set("Power:")
        power_label=Label(self.typing_frame, textvariable=power_text, height=4)
        power_label.pack(side="left")
        self.typing_power = Entry(self.typing_frame)
        self.typing_power.pack(side=LEFT)

        alpha_text=StringVar()
        alpha_text.set("Alpha:")
        alpha_label=Label(self.typing_frame, textvariable=alpha_text, height=4)
        alpha_label.pack(side="left")
        self.typing_alpha = Entry(self.typing_frame)
        self.typing_alpha.pack(side=RIGHT)
       
        # add motor control slider
        self.forward_control = Scale(self.bottom_frame, from_=-100, to=100, length  = 400,label="Forward Control",tickinterval=50, orient=HORIZONTAL, resolution=CONFIG_SCALE_RESOLUTION)
        self.forward_control.pack(side=LEFT)
        
        # add rotation control slider
        self.rotate_control = Scale(self.bottom_frame, from_=-100, to=100, length  = 400,label="Rotate Control",tickinterval=50, orient=HORIZONTAL, resolution=CONFIG_SCALE_RESOLUTION)
        self.rotate_control.pack(side=RIGHT)
        
        # add track point button
        self.track_point_button = Button(self.bottom_frame, text="Track Point", anchor="s", wraplength=100, command=self.track_point).pack()
        
        # add stop button
        self.track_point_button = Button(self.bottom_frame, text="Stop", anchor="s", wraplength=100, command=self.stop).pack()
  
        # add stop button
        self.track_point_button = Button(self.bottom_frame, text="Quit", anchor="s", wraplength=100, command=self.quit).pack()
  
        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)
            
        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r) 

        # add range sensor measurements
        self.range_sensor_var_1 = StringVar()
        self.range_sensor_var_2 = StringVar()
        self.range_sensor_var_3 = StringVar()
        self.range_sensor_label_1 = Label(self.north_east_frame, textvariable = self.range_sensor_var_1).pack()
        self.range_sensor_label_2 = Label(self.north_east_frame, textvariable = self.range_sensor_var_2).pack()
        self.range_sensor_label_3 = Label(self.north_east_frame, textvariable = self.range_sensor_var_3).pack()

        # add encoder sensor measurements
        self.encoder_sensor_var_0 = StringVar()
        self.encoder_sensor_var_1 = StringVar()
        
        self.encoder_sensor_label_0 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_0).pack()
        self.encoder_sensor_label_1 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_1).pack()

        # add range sensor measurements
        self.x = StringVar()
        self.y = StringVar()
        self.theta = StringVar()
        self.x_label = Label(self.north_east_frame, textvariable = self.x).pack()
        self.y_label = Label(self.north_east_frame, textvariable = self.y).pack()
        self.theta_label = Label(self.north_east_frame, textvariable = self.theta).pack()
       
    
        
        
        #self.range_sensor_label_1.pack()   
    
    

    def draw_wall(self, wall):
        
        wall_points = self.scale_points(wall.points, self.scale)
        wall.poly = self.canvas.create_polygon(wall_points, fill='black')
        
    def scale_points(self, points, scale):
        scaled_points = []
        for i in range(len(points)-1):
            
            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                scaled_points.append(self.environment.width/2*scale + points[i]*scale)   
                
                # only flip y for x,y points, not for circle radii
                scaled_points.append(self.environment.height/2*scale - points[i+1]*scale)   
                    
        return scaled_points
    
    
    def reverse_scale_points(self, points, scale):
        reverse_scaled_points = []
        for i in range(len(points)-1):
            
            if i % 2 == 0:
                # for x values, just multiply times scale factor to go from meters to pixels
                reverse_scaled_points.append(-self.environment.width/2 + points[i]/scale)   
                
                # only flip y for x,y points, not for circle radii
                reverse_scaled_points.append(self.environment.height/2 - points[i+1]/scale)   
                    
        return reverse_scaled_points
    
    
    def initial_draw_robot(self, robot):
            
        # open image
        robot.robot_gif = Image.open("E160_robot_image.gif").convert('RGBA') 

        
    def draw_robot(self, robot):
        
        # gif update
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180/3.14*robot.state_est.theta))
        robot.image = self.canvas.create_image(robot.state_est.x, robot.state_est.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state_est.x, robot.state_est.y], self.scale)
        self.canvas.coords(robot.image, *robot_points)
            
    def get_inputs(self):
        pass
     

        
    def track_point(self):
        self.environment.control_mode = "AUTONOMOUS CONTROL MODE"
                
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.last_forward_control = 0
        self.last_rotate_control = 0
        self.R = 0
        self.L = 0
        
    def stop(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"
        
        # update sliders on gui
        self.forward_control.set(0)
        self.rotate_control.set(0)
        self.typing_power.delete(0,END)
        self.typing_power.insert(0,0)      
        self.last_forward_control = 0
        self.last_rotate_control = 0  
        self.R = 0
        self.L = 0
        
    def quit(self):
        self.environment.control_mode = "MANUAL CONTROL MODE"
        self.forward_control.set(0)
        self.rotate_control.set(0) 
        self.typing_power.delete(0,END)
        self.typing_power.insert(0,0)
        self.gui_stopped = True
        
        
    def callback(self, event):
        desired_points = self.reverse_scale_points([float(event.x), float(event.y)], self.scale)
        robot = self.environment.robots[0]
        robot.state_des.set_state(desired_points[0],desired_points[1],0)
        print("New desired robot state", robot.state_des.x, robot.state_des.y)
        
        
    def send_robot_commands(self):
        

        # check to see if forward slider has changed
        if abs(self.forward_control.get()-self.last_forward_control) > 0:
            self.rotate_control.set(0)       
            self.last_forward_control = self.forward_control.get()
            self.last_rotate_control = 0         
            self.environment.control_mode = "MANUAL CONTROL MODE"
            
            # extract what the R and L motor signals should be
            self.R = self.forward_control.get()
            self.L = self.forward_control.get()
  
        # check to see if rotate slider has changed
        elif abs(self.rotate_control.get()-self.last_rotate_control) > 0:
            self.forward_control.set(0)       
            self.last_rotate_control = self.rotate_control.get()
            self.last_forward_control = 0         
            self.environment.control_mode = "MANUAL CONTROL MODE"
        
            # extract what the R and L motor signals should be
            self.R = -self.rotate_control.get()
            self.L = self.rotate_control.get()
        
        # if manual mode, set motors
        if self.environment.control_mode == "MANUAL CONTROL MODE":
            
            # tell robot what the values should be
            robot = self.environment.robots[0]
            robot.set_manual_control_motors(self.R, self.L)
        
        
        
    def update_labels(self):
        
        self.range_sensor_var_1.set("Range 1 (m):  " + str(self.environment.robots[0].range_measurements[0]))
        self.range_sensor_var_2.set("Range 2 (m):  " + str(self.environment.robots[0].range_measurements[1]))
        self.range_sensor_var_3.set("Range 3 (m):  " + str(self.environment.robots[0].range_measurements[2]))
                
        self.encoder_sensor_var_0.set("Encoder 0 (m):  " + str(self.environment.robots[0].encoder_measurements[0]))
        self.encoder_sensor_var_1.set("Encoder 1 (m):  " + str(self.environment.robots[0].encoder_measurements[1]))

        self.x.set("X (m):  " + str(self.environment.robots[0].state_est.x))
        self.y.set("Y (m):  " + str(self.environment.robots[0].state_est.y))
        self.theta.set("Theta (rad):  " + str(self.environment.robots[0].state_est.theta))  

    # called at every iteration of main loop
    def update(self):
        
        # update gui labels
        self.update_labels()
        
        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)     
        
        # draw particles
        
        
        # draw sensors
        
        # check for text input
        try: 
            typing_power=int(self.typing_power.get())
        except:
            typing_power=0

        
        self.forward_control.set(typing_power)

            

        # update the graphics
        self.tk.update()

        # check for gui buttons
        self.get_inputs()
        
        # send commands to robots
        self.send_robot_commands()
        
        # check for quit
        if self.gui_stopped:
            self.environment.quit()
            return False
        else:
            return True
        
        
        
    
    
    
   




