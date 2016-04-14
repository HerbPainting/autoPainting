import numpy
from RRTTree import RRTTree
from random import random
from random import seed

class RRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.scale = 5.0
        self.unit_change = 100.0
        seed(0)
        

    def Plan(self, start_config_list, goal_config_list, epsilon = 0.001):
        
        
        plan = []
        draw_plan = []
        
        self.planning_env.InitializePlot()
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        #plan.append(start_config)
        #new_config = start_config_list[0]
        
        #while  distance > epsilon and len(draw_plan) < 200:
        N = 4
        for i in xrange(N):
            print "Pass: ", i+1
            
            distance = 1000
            
            start_config = start_config_list[i]
            goal_config = goal_config_list[i]
            tree = RRTTree(self.planning_env, start_config)
            while distance > epsilon and len(tree.vertices) < 20:
                
                # r = random()
                # if r > self.planning_env.p:
                target_config = self.planning_env.GenerateRandomConfiguration()
                # else:
                    # target_config = goal_config
                current_id, current_config = tree.GetNearestVertex(target_config)
                new_config = self.planning_env.Extend(current_config, target_config)
                if new_config == None:
                    continue
                else:
                                        
                    if new_config[0] < 0.1 or new_config[0] >= (self.planning_env.boundary_limits[1][0] - 0.1) or new_config[1] < 0.1 or new_config[1] >= (self.planning_env.boundary_limits[1][1] - 0.1):
                    	continue
                    new_id = tree.AddVertex(new_config)
                    pt1 = [x*self.scale/self.unit_change for x in tree.vertices[current_id]]
                    pt2 = [x*self.scale/self.unit_change for x in tree.vertices[new_id]]
                    draw_plan.append((pt1,pt2)) #of the form (prev,next)
                    tree.AddEdge(current_id, new_id)
                    self.planning_env.PlotEdge(tree.vertices[current_id], tree.vertices[new_id])
                    distance = self.planning_env.ComputeDistance(new_config, goal_config)
                   
            if distance <= epsilon:
                print "reached"
        # while new_id != 0:
        #     new_id = tree.edges[new_id] 
        #     prev_node = tree.vertices[new_id]
        #     plan.insert(0,prev_node)
        
        # plan.append(goal_config)
        # Filter draw plan to remove points too near to the edge of the canvas

        return draw_plan
