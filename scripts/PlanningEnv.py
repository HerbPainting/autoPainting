import numpy
import matplotlib.pyplot as pl
import random
import math
from time import time
from math import cos, sin, floor, ceil
class PlanningEnv(object):
    
    def __init__(self):
        
        self.boundary_limits = numpy.array([[0., 0.], [20., 20.]])
        #TODO: Load Costmap
        self.raster_size = 0.1
        costmapcol = int(self.boundary_limits[1][1]/self.raster_size)
        costmaprow = int(self.boundary_limits[1][0]/self.raster_size)
        self.costmap = [[0 for x in range(costmapcol)] for x in range(costmaprow)]

        # TODO: Add obstacles
        self.obstacles = []
        self.strokeLength = 4

        # self.costmap[1][1] = 255
        # self.costmap[1][2] = 255e
        # self.costmap[2][1] = 255
        # self.costmap[2][2] = 255
        
        # self.obstacles.append([1,1])
        # self.obstacles.append([1,2])
        # self.obstacles.append([2,1])
        # self.obstacles.append([2,2])
        # Square in center obstacle for resolution = 1############
        # for i in xrange(7,14):
        #     for j in xrange(7,14):
        #         self.costmap[i][j] = 255
        #         self.obstacles.append([i,j])
        ###############################
        # self.setObstacles([ [(8,8),(12,12l)] ])
        drawlines  = []
        #Supposed smilie face
        #eye1
        drawlines.append([(5,16),(7,16)])
        drawlines.append([(5,16),(5,11)])
        drawlines.append([(7,16),(7,11)])
        drawlines.append([(5,11),(7,11)])
        #eye2
        drawlines.append([(13,16),(15,16)])
        drawlines.append([(13,16),(13,11)])
        drawlines.append([(15,16),(15,11)])
        drawlines.append([(13,11),(15,11)])
        #nose
        drawlines.append([(9,12),(11,12)])
        drawlines.append([(9,12),(9,8)])
        drawlines.append([(11,12),(11,8)])
        drawlines.append([(9,8),(11,8)])
        #mouth
        drawlines.append([(6,6),(14,6)])
        drawlines.append([(6,6),(6,4)])
        drawlines.append([(14,6),(14,4)])
        drawlines.append([(6,4),(14,4)])


        self.setObstacles(drawlines)


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

    def ComputeHeuristicCost(self,start_config, goal_config):

        return numpy.linalg.norm(numpy.asarray(goal_config) - numpy.asarray(start_config))

    def Extend(self, start_config, end_config,extendSize):
        
        #   A function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        dist = self.ComputeDistance(start_config, end_config)
        number_of_steps = int(dist*40)
        x = numpy.linspace(start_config[0], end_config[0], number_of_steps)
        y = numpy.linspace(start_config[1], end_config[1], number_of_steps)
        config_to_return = None
        for i in xrange(1,number_of_steps):
            temp_config = numpy.array([x[i], y[i]])
            inCollision = self.collision_checker(temp_config)            
            if inCollision:
                return config_to_return
            else:
    			config_to_return = temp_config
    	return config_to_return
        # m = math.atan2(end_config[1] - start_config[1],end_config[0] - start_config[0])
        # dMax = extendSize
        # totalDistance = numpy.linalg.norm(end_config - start_config)
        # d = 0
        # while d < dMax and d <= totalDistance:
        #     x2 = start_config[0] + d*math.cos(m)
        #     y2 = start_config[1] + d*math.sin(m)
        #     temp_config = numpy.array([x2,y2])
        #     if x2 < 0 or x2 >= (len(self.costmap[0])) or y2 < 0 or y2 >= (len(self.costmap)):
        #         return None 
        #     inCollision = self.collision_checker(temp_config)
        #     if inCollision:
        #         return None
        #     d += self.raster_size*0.1
        # if inCollision:
        #     return None
        # else:
        #     return temp_config

        
    def setObstacles(self, lines):
        
        for line in lines:
            inflatedLines = []
            pt1 = numpy.asarray(line[0])
            pt2 = numpy.asarray(line[1])
            length = numpy.linalg.norm(pt2 - pt1)
            l = 0.0
            deltaLength = self.raster_size
            m = math.atan2(pt2[1]-pt1[1],pt2[0]-pt1[0])
            #Inflate the obstacles by adding lines parallel to the specified line at intervales of raster_size and upto strokeLength
            for i in xrange(-self.strokeLength, self.strokeLength + 1):
                for j in xrange(-self.strokeLength, self.strokeLength + 1):
                    inflatedLines.append([ (pt1[0] + i*(self.raster_size/2)*(-1.0*sin(m)), pt1[1] + j*(self.raster_size/2)*cos(m)), (pt2[0] + i*(self.raster_size/2)*(-1.0*sin(m)), pt2[1] + j*(self.raster_size/2)*cos(m)) ])
            # inflatedLines = line

            #Now add obstacles along the inflated lines
            for inflatedLine in inflatedLines:
                pt1i = numpy.asarray(inflatedLine[0])
                pt2i = numpy.asarray(inflatedLine[1])
                lengthi = numpy.linalg.norm(pt2i - pt1i)
                li = 0.0
                deltaLengthi = self.raster_size
                mi = math.atan2(pt2i[1]-pt1i[1],pt2i[0]-pt1i[0])
                while li < lengthi:
                    newConfig = numpy.array([pt1i[0] + li*cos(mi),pt1i[1] + li*sin(mi)])
                    discreteConf = self.configurationToGridCoordinate(newConfig)
                    self.costmap[discreteConf[1]][discreteConf[0]] = 255
                    self.obstacles.append(discreteConf)
                    li += deltaLengthi


    # def ExtendHrrt()

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
            obsConfig = self.gridCoordinateToConfiguration(obs)
            plot_x.append(obsConfig[0])
            plot_y.append(obsConfig[1])
        pl.plot(plot_x,plot_y,'r.')
               
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k', linewidth=1)
        pl.draw()
    def getBoundaryLimits(self):
        return self.boundary_limits

    def configurationToGridCoordinate(self,config):
        config = numpy.asarray(config)
        discreteConf = numpy.array([int(ceil(config[0]/self.raster_size)), int(ceil(config[1]/self.raster_size))])
        return discreteConf
    def gridCoordinateToConfiguration(self,coord):
        coord = numpy.asarray(coord)
        m = [self.raster_size]*2
        config = numpy.multiply(numpy.add(coord,m),self.raster_size)
        return config
