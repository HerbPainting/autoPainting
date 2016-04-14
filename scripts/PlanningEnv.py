import numpy
import matplotlib.pyplot as pl
import random
import math
from time import time

class PlanningEnv(object):
    
    def __init__(self):
        
        self.boundary_limits = numpy.array([[0., 0.], [4., 4.]])
        #TODO: Load Costmap
        self.raster_size = 1
        costmapcol = int(self.boundary_limits[1][1]/self.raster_size)
        costmaprow = int(self.boundary_limits[1][0]/self.raster_size)
        self.costmap = [[0 for x in range(costmapcol)] for x in range(costmaprow)]

        # TODO: Add obstacles
        self.obstacles = []
        # for i in range(2):
        #     for j in range(1):
        #         self.costmap[i + costmaprow/2][j+costmapcol/2] = 255
        #         self.obstacles.append([(i + costmaprow/2)*self.raster_size, (j+costmapcol/2)*self.raster_size ])
        self.costmap[1][1] = 255
        self.costmap[1][2] = 255
        self.costmap[2][1] = 255
        self.costmap[2][2] = 255
        
        self.obstacles.append([1,1])
        self.obstacles.append([1,2])
        self.obstacles.append([2,1])
        self.obstacles.append([2,2])
        # goal sampling probability
        self.p = 0.1

    def SetGoalParameters(self, goal_config, p = 0.1):
        self.goal_config = goal_config
        self.p = p
        
    
    def collision_checker(self,config):
    	x = int(config[0]/self.raster_size)
    	y = int(config[1]/self.raster_size)
    	return self.costmap[y][x] == 255



    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # Generate and return a random configuration
        #
        inCollision = True
        while inCollision:
        	config[0] = random.uniform(lower_limits[0], upper_limits[0])
        	config[1] = random.uniform(lower_limits[1], upper_limits[1])
        	inCollision = self.collision_checker(config)         	        
        
        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        #
        # A function which computes the distance between
        # two configurations
        #

        return numpy.linalg.norm(numpy.asarray(end_config) - numpy.asarray(start_config))

    def Extend(self, start_config, end_config):
        
        #   A function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        
    	 # dist = self.ComputeDistance(start_config, end_config)
    	 # number_of_steps = int(dist*20)
    	 # x = numpy.linspace(start_config[0], end_config[0], number_of_steps)
    	 # y = numpy.linspace(start_config[1], end_config[1], number_of_steps)
    	# config_to_return = None

    	# for i in xrange(1,number_of_steps):
    	# 	temp_config = numpy.array([x[i], y[i]])
    	# 	inCollision = self.collision_checker(temp_config)
    	# 	if inCollision:
    	# 		return config_to_return
    	# 	else:
    	# 		config_to_return = temp_config
    	# return config_to_return
        m = math.atan2(end_config[1] - start_config[1],end_config[0] - start_config[0])
        d = 0.5
        x2 = start_config[0] + d*math.cos(m)
        y2 = start_config[1] + d*math.sin(m)
        temp_config = numpy.array([x2,y2])
        if x2 < 0 or x2 >= (len(self.costmap[0])) or y2 < 0 or y2 >= (len(self.costmap)):
            return None 
        inCollision = self.collision_checker(temp_config)
        if inCollision:
            return None
        else:
            return temp_config

        

    # def ShortenPath(self, path, timeout=5.0):
        
    #     # 
    #     # A function which performs path shortening
    #     #  on the given path.  Terminate the shortening after the 
    #     #  given timout (in seconds).
    #     #
    #     t = time()
    #     while time() - t < timeout:
    #         idx1 = random.randint(0,len(path)-1)
    #         idx2 = random.randint(idx1,len(path)-1)
    #         q_new = self.Extend(path[idx1], path[idx2])
    #         if q_new != None:
    #             if numpy.array_equal(q_new,path[idx2]):
    #                 path[idx1+1:idx2] = []
    #     return path


    def InitializePlot(self):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        #pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        """for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')"""
        plot_x = []
        plot_y = []
        for obs in self.obstacles:
            
            plot_x.append(obs[0])
            plot_y.append(obs[1])
        #pl.plot(plot_x,plot_y,'ro')
               
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k', linewidth=1)
        pl.draw()
    def getBoundaryLimits(self):
        return self.boundary_limits