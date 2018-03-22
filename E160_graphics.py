from E160_config import *
import math
import sys
# Use correct package given version of python
if sys.version_info[0] < 3:
    from Tkinter import *
else:
    from tkinter import *
from E160_robot import *
from PIL import Image, ImageTk


class E160_graphics:
    
    def __init__(self, environment):
        self.environment = environment
        self.tk = Tk()
        #self.top_frame = Frame(self.tk) #blame clark
        #self.top_frame.pack(anchor = N)
        #self.north_east_frame = Frame(self.tk)
        #self.north_east_frame.pack(anchor = NE)
        self.north_west_frame = Frame(self.tk)
        self.north_west_frame.pack(anchor = W)

        self.east_north_west_frame = Frame(self.tk)

        self.east_north_west_frame.pack(anchor = E)
        self.north_frame = Frame(self.north_west_frame)
        self.north_frame.pack(side='right')
        self.north_easter_frame = Frame(self.north_frame)
        self.north_easter_frame.pack(side='right')

        self.bottom_frame = Frame(self.tk)
        self.bottom_frame.pack(side = BOTTOM)

        self.typing_frame = Frame(self.bottom_frame)
        self.typing_frame.pack(side = TOP)
        self.prev_typing_int = 0
        self.typing_int = 0

        self.scale = CONFIG_WINDOW_SCALE
        self.canvas = Canvas(self.tk, width=self.environment.width*self.scale*2, height=self.scale*self.environment.height*2)
        self.tk.title("E160 - Autonomous Robot Navigation")
        self.canvas.bind("<Button-1>", self.callback)
        self.canvas.pack()
        self.gui_stopped = False
        self.last_rotate_control = 0
        self.last_forward_control = 0
        self.R = 0
        self.L = 0

        self.robot = self.environment.robots[0]


        
        # attempt at typing input
        power_text=StringVar()
        power_text.set("Power:")
        power_label=Label(self.typing_frame, textvariable=power_text, height=4)
        power_label.pack(side="left")
        self.typing_power = Entry(self.typing_frame)
        self.typing_power.pack(side=LEFT)

        proportional=StringVar()
        proportional.set("Kp:")
        proportional_label=Label(self.typing_frame, textvariable=proportional, height=4)
        proportional_label.pack(side="left")
        self.typing_proportional = Entry(self.typing_frame)
        self.typing_proportional.pack(side=LEFT)

        k_alpha_text=StringVar()
        k_alpha_text.set("Ka:")
        k_alpha_label=Label(self.typing_frame, textvariable=k_alpha_text, height=4)
        k_alpha_label.pack(side="left")
        self.typing_k_alpha = Entry(self.typing_frame)
        self.typing_k_alpha.pack(side=LEFT)

        k_beta_text=StringVar()
        k_beta_text.set("Kb:")
        k_beta_label=Label(self.typing_frame, textvariable=k_beta_text, height=4)
        k_beta_label.pack(side="left")
        self.typing_k_beta = Entry(self.typing_frame)
        self.typing_k_beta.pack(side=LEFT)
       
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
  


        # add range sensor measurements
        self.range_sensor_var_1 = StringVar()
        self.range_sensor_var_2 = StringVar()
        self.range_sensor_var_3 = StringVar()
        self.range_sensor_label_1 = Label(self.north_west_frame, textvariable = self.range_sensor_var_1).pack()
        self.range_sensor_label_2 = Label(self.north_west_frame, textvariable = self.range_sensor_var_2).pack()
        self.range_sensor_label_3 = Label(self.north_west_frame, textvariable = self.range_sensor_var_3).pack()

        # add encoder sensor measurements
        self.encoder_sensor_var_0 = StringVar()
        self.encoder_sensor_var_1 = StringVar()
        
        self.encoder_sensor_label_0 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_0).pack()
        self.encoder_sensor_label_1 = Label(self.north_west_frame, textvariable = self.encoder_sensor_var_1).pack()

        # add range sensor measurements
        self.x = StringVar()
        self.y = StringVar()
        self.theta = StringVar()
        self.x_label = Label(self.north_frame, textvariable = self.x).pack()
        self.y_label = Label(self.north_frame, textvariable = self.y).pack()
        self.theta_label = Label(self.north_frame, textvariable = self.theta).pack()

        self.x_real = StringVar()
        self.y_real = StringVar()
        self.theta_real = StringVar()
        self.x_real_label = Label(self.north_easter_frame, textvariable = self.x_real, anchor='e').pack()
        self.y_real_label = Label(self.north_easter_frame, textvariable = self.y_real, anchor='e').pack()
        self.theta_real_label = Label(self.north_easter_frame, textvariable = self.theta_real, anchor='e').pack()
       
        # add text entry for desired X
        #self.x_des_label = Label(self.north_frame, text="X desired")
        #self.x_des_label.pack()
        self.x_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.x_des_entry.insert(10,"0.0")
        self.x_des_entry.pack()
        
        # add text entry for desired Y
        #self.y_des_label = Label(self.north_west_frame, text="Y desired")
        #self.y_des_label.pack()
        self.y_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.y_des_entry.insert(10,"0.0")
        self.y_des_entry.pack()
        
        # add text entry for desired Theta
        #self.theta_des_label = Label(self.north_west_frame, text="Theta desired")
        #self.theta_des_label.pack()
        self.theta_des_entry = Entry(self.north_west_frame, justify = RIGHT)
        self.theta_des_entry.insert(10,"0.0")
        self.theta_des_entry.pack()


        self.typing_proportional.insert(0,self.robot.K_rho)
        self.typing_k_alpha.insert(0,self.robot.K_alpha) 
        self.typing_k_beta.insert(0,self.robot.K_beta)  

        # initilize particle representation
        self.particles_dot = [self.canvas.create_oval(0,0,0,0, fill ='black') for x in range(self.environment.robots[0].PF.numParticles)]

        # draw static environment
        for w in self.environment.walls:
            self.draw_wall(w)
            
        # draw first robot
        for r in self.environment.robots:
            self.initial_draw_robot(r)    
     
  
    
    

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
        robot.tkimage = ImageTk.PhotoImage(robot.robot_gif.rotate(180/3.14*robot.state_draw.theta))
        robot.image = self.canvas.create_image(robot.state_draw.x, robot.state_draw.y, image=robot.tkimage)
        robot_points = self.scale_points([robot.state_draw.x, robot.state_draw.y], self.scale)
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

        # Get the desired point
        for r in self.environment.robots:
            x_des = float(self.x_des_entry.get())
            y_des = float(self.y_des_entry.get())
            theta_des = float(self.theta_des_entry.get())
            r.state_des.set_state(x_des,y_des,theta_des)
            r.point_tracked = False 
        
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
        delta_forward = self.forward_control.get()-self.last_forward_control
        if abs(delta_forward) > 0:
            #ramped_delta = max(min(delta_forward,CONFIG_RAMP_CONSTANT),-CONFIG_RAMP_CONSTANT)
            self.rotate_control.set(0)
            self.last_forward_control = self.forward_control.get()
            #self.last_forward_control = self.last_forward_control + ramped_delta #why did I do this in gfx/
            self.last_rotate_control = 0         
            self.environment.control_mode = "MANUAL CONTROL MODE"
            
            # extract what the R and L motor signals should be
            # self.R = self.forward_control.get()
            # self.L = self.forward_control.get()
            self.R = self.forward_control.get() # last forward control is currently
            self.L = self.forward_control.get() #  this forward control...

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
                
        self.encoder_sensor_var_0.set("Encoder 0 (ticks L):  " + str(self.environment.robots[0].encoder_measurements[0]))
        self.encoder_sensor_var_1.set("Encoder 1 (ticks R):  " + str(self.environment.robots[0].encoder_measurements[1]))

        self.x.set("X (m):  " + str(self.environment.robots[0].state_est.x))
        self.y.set("Y (m):  " + str(self.environment.robots[0].state_est.y))
        self.theta.set("Theta (rad):  " + str(self.environment.robots[0].state_est.theta))  

        self.x_real.set("X (m):  " + str(self.environment.robots[0].state_odo.x))
        self.y_real.set("Y (m):  " + str(self.environment.robots[0].state_odo.y))
        self.theta_real.set("Theta (rad):  " + str(self.environment.robots[0].state_odo.theta))  


    # called at every iteration of main loop
    def update(self):
        
        # update gui labels
        self.update_labels()
        
        # draw robots
        for r in self.environment.robots:
            self.draw_robot(r)     
        
        # draw particles
        self.draw_particles(self.environment.robots[0])
        
        # draw sensors

        
        # check for text input
        self.prev_typing_int = self.typing_int
        try: 
            self.typing_int = int(self.typing_power.get())
            self.typing_int = min(100,self.typing_int)
            self.typing_int = max(-100,self.typing_int)
        except:
            typing_int=0
        try:
            self.k_proportional = float(self.typing_proportional.get())
            self.k_alpha = float(self.typing_k_alpha.get())
            self.k_beta = float(self.typing_k_beta.get())
            self.robot.K_rho = self.k_proportional
            self.robot.K_alpha = self.k_alpha
            self.robot.K_beta = self.k_beta
        except:
            print('error getting some K')

        if(self.prev_typing_int != self.typing_int):
            self.forward_control.set(self.typing_int)

            

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

    def draw_particles(self, robot):
        for i in range(robot.PF.numParticles):
            pf_point = [robot.PF.particles[i].x, robot.PF.particles[i].y]
            point = self.scale_points(pf_point, self.scale)
            self.canvas.delete(self.particles_dot[i]) 
            self.particles_dot[i] = self.canvas.create_oval(point[0] - 2, point[1] - 2, point[0] + 2, point[1] + 2, fill =  'red')
        