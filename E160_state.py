

class E160_state:

    def __init__(self):
        self.set_state(0,0,0)
        self.theta_cumulative = 0
        
    def set_state(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def add_theta(self, delta_theta):
    	self.theta_cumulative += delta_theta
    	print(self.theta_cumulative)

    def __str__(self):
      return " ".join([str(self.x), str(self.y), str(self.theta)])
      