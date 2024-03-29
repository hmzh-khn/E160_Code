import math
from E160_config import CONFIG_DEGS_PER_REVOLUTION

class E160_state:

    def __init__(self, x=0, y=0, theta=0):
        if isinstance(x, tuple):
            x, y, theta = x

        self.set_state(x, y, theta)
        self.theta_cumulative = 0
        
    def set_state(self,x,y=None,theta=None):
        # print('y: ', y, '\n')
        # if (y != []) and (y != 0):
        #     # is another E160_state
        #     self.y = x.y
        #     self.theta = x.theta
        #     self.x = x.x
        # else:
        self.x = x
        self.y = y
        self.theta = theta

    def add_theta(self, delta_theta):
    	self.theta_cumulative += delta_theta

    def get_state_difference(self, destination_state):
        delta_x = self.x - destination_state.x
        delta_y = self.y - destination_state.y
        delta_theta = self.normalize_angle(self.theta - destination_state.theta)
        delta_theta = self.short_angle(delta_theta)
        return E160_state(delta_x, delta_y, delta_theta)

    def __str__(self):
        return " ".join([str(self.x), str(self.y), str(self.theta)])
    def __repr__(self):
        return "[" + str(self.x) + " " + str(self.y) + " " + str(self.theta) + "]"

    def normalize_angle(self, theta):
        '''
        Returns an angle in range (-pi, pi]
        '''
        out_angle = theta % (2 * math.pi)
        if out_angle > math.pi:
            out_angle = out_angle - 2 * math.pi
        return out_angle
      
    #concern could mess up the final orientaton
    def short_angle(self,theta):
        if theta > math.pi:
            theta = theta - 2 * math.pi
        if theta < -math.pi:
            theta = 2*math.pi + theta
        return theta

    def __add__(self, other):
        return E160_state(self.x + other.x, self.y + other.y, self.normalize_angle(self.theta + other.theta))

    def __sub__(self, other):
        return E160_state(self.x - other.x, self.y - other.y, self.normalize_angle(self.theta - other.theta))

    def xydist(self, other):
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)
