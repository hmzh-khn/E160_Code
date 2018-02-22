import math
from E160_config import CONFIG_DEGS_PER_REVOLUTION

class E160_state:

    def __init__(self, x=0, y=0, theta=0):
        self.set_state(x, y, theta)
        self.theta_cumulative = 0
        
    def set_state(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def add_theta(self, delta_theta):
    	self.theta_cumulative += delta_theta
    	print(self.theta_cumulative, self.theta_cumulative/ (2 * math.pi) * CONFIG_DEGS_PER_REVOLUTION)

    def get_state_difference(self, destination_state):
        delta_x = self.x - destination_state.x
        delta_y = self.y - destination_state.y
        delta_theta = self.normalize_angle(self.theta - destination_state.theta)
        return E160_state(delta_x, delta_y, delta_theta)

    def __str__(self):
        return " ".join([str(self.x), str(self.y), str(self.theta)])

    def normalize_angle(self, theta):
        '''
        Returns an angle in range (-pi, pi]
        '''
        out_angle = theta % (2 * math.pi)
        if out_angle > math.pi:
            out_angle = out_angle - 2 * math.pi
        return out_angle
      