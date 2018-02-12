

class E160_state:

    def __init__(self):
        self.set_state(0,0,0)
        
    def set_state(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
      return str((self.x, self.y, self.theta))
      