import numpy
from RRTTree import RRTTree
from random import random
from random import seed

class mixedRRTPlanner(object):

    def __init__(self, planning_env,extendSize):
        self.planning_env = planning_env
        self.scale = 1.0
        self.unit_change = 100.0
        
        self.extendSize = extendSize
        self.boundary_limits = planning_env.getBoundaryLimits()


    def Plan(self, start_config_list, goal_config_list, epsilon = 0.001):
        
        
        plan = []
        draw_plan = []
        
        self.planning_env.InitializePlot()
        #plan.append(start_config)
        #new_config = start_config_list[0]
        
        #while  distance > epsilon and len(draw_plan) < 200:
        N = 4
        for i in xrange(N):
            print "Pass: ", i+1
            
            
            
            start_config = start_config_list[i]
            goal_config = goal_config_list[i]
            tree = RRTTree(self.planning_env, start_config)
            while len(tree.vertices) < 20:
                
                r = random()
                if r < self.planning_env.coveragep:
                	coverage = True
                else:
                	coverage = False
                target_config = self.planning_env.GenerateRandomConfiguration(coverage)
             	
                current_id, current_config = tree.GetNearestVertex(target_config)
                #Keep sampling till large length is not obtained
                count = 0
                while self.planning_env.ComputeDistance(current_config,target_config) < (self.extendSize) and count < len(tree.vertices) and not coverage:
                	current_id, current_config = tree.GetithNearestVertex(target_config,count)
                	count += 1
                
                new_config = self.planning_env.Extend(current_config, target_config,self.extendSize)
                if new_config == None:
                    continue
                else:
                                        
                    if new_config[0] < (self.boundary_limits[0][0]+0.1) or new_config[0] >= (self.boundary_limits[1][0] - 0.1) or new_config[1] < (self.boundary_limits[0][1]+0.1) or new_config[1] >= (self.boundary_limits[1][1] - 0.1):
                    	continue
                    new_id = tree.AddVertex(new_config)
                    pt1 = [x*self.scale/self.unit_change for x in tree.vertices[current_id]]
                    pt2 = [x*self.scale/self.unit_change for x in tree.vertices[new_id]]
                    draw_plan.append((pt1,pt2)) #of the form (prev,next)
                    tree.AddEdge(current_id, new_id)
                    self.planning_env.PlotEdge(tree.vertices[current_id], tree.vertices[new_id])
                


        return draw_plan