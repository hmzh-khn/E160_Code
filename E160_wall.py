class E160_wall:

    
    def __init__(self, wall_points, slope):
        
        # set up walls
        self.slope = slope
        self.radius = 0.025
        self.wall_points = wall_points
        self.point1 = (wall_points[0], wall_points[1])
        self.point2 = (wall_points[2], wall_points[3])
        
        # assume top point is first
        if slope == "vertical":
            self.points = [wall_points[0]-self.radius, wall_points[1]+self.radius,
                    wall_points[0]+self.radius, wall_points[1]+self.radius,
                    wall_points[2]+self.radius, wall_points[3]-self.radius,
                    wall_points[2]-self.radius, wall_points[3]-self.radius]
        
        # assume left point is first
        elif slope == "horizontal":
            self.points = [wall_points[0]-self.radius, wall_points[1]-self.radius,
                    wall_points[0]-self.radius, wall_points[1]+self.radius,
                    wall_points[2]+self.radius, wall_points[3]+self.radius,
                    wall_points[2]+self.radius, wall_points[3]-self.radius]

    def contains_point(self, p):
        x, y = p
        p1x, p1y = self.point1
        p2x, p2y = self.point2
        min_x = min(p1x, p2x)
        max_x = max(p1x, p2x)
        min_y = min(p1y, p2y)
        max_y = max(p1y, p2y)
        return x <= max_x and x >= min_x  and y >= min_y and y <= max_y
        
    def slope_intercept(self):
        p1x, p1y = self.point1
        p2x, p2y = self.point2
        slope = (p2y - p1y) / (p2x - p1x)
        intercept = p1y - slope * p1x 
        return (slope, intercept)
        
