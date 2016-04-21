import numpy
from HRRTTree import HRRTTree
from random import random, Random
import time

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env):
        self.planning_env = planning_env
        self.r1 = Random()
        self.r2 = Random()
        self.scale = 1.0
        self.unit_change = 100.0
        

    def Plan(self, start_config_list, goal_config_list, epsilon = 0.001):
        
        
        plan = []
        draw_plan = []
        self.planning_env.InitializePlot()

        
        N = 4
        for i in xrange(N):
            print "Pass: ", i+1       
            start_config = start_config_list[i]
            goal_config = goal_config_list[i]
            copt = self.planning_env.ComputeDistance(start_config, goal_config)
            tree = HRRTTree(self.planning_env, start_config,0, self.planning_env.ComputeHeuristicCost(start_config, goal_config) + 0)
            distance = 100
            while  distance > epsilon and len(tree.vertices) < 40:
            
                if self.r1.random() >= self.planning_env.p:
                
                    current_id, current_config, curr_cost, curr_total_cost, target_config = self.selectNode(tree, copt)
            
                else:
                    target_config = goal_config
               
                    current_id, current_config, curr_cost, curr_total_cost = tree.GetNearestVertex(target_config)

            
            #tree.GetNearestVertex(target_config)
                new_config,new_cost,new_total_cost = self.planning_env.Extend(current_config, target_config, curr_cost, goal_config)
            
                if new_config == None:
                    continue
                else:
                    if new_config[0] < 0.1 or new_config[0] >= (self.planning_env.boundary_limits[1][0] - 0.1) or new_config[1] < 0.1 or new_config[1] >= (self.planning_env.boundary_limits[1][1] - 0.1):
                        continue
                    new_id = tree.AddVertex(new_config,new_cost,new_total_cost)
                    pt1 = [x*self.scale/self.unit_change for x in tree.vertices[current_id]]
                    pt2 = [x*self.scale/self.unit_change for x in tree.vertices[new_id]]
                    draw_plan.append((pt1,pt2))
                    tree.AddEdge(current_id, new_id)
                    self.planning_env.PlotEdge(tree.vertices[current_id], tree.vertices[new_id])
                    distance = self.planning_env.ComputeDistance(new_config, goal_config)

            if distance <= epsilon:
                print "reached"
                
        # plan = []
        # print "pathlength", tree.costs[-1]
        # while new_id != 0:
        #     new_id = tree.edges[new_id] 
        #     prev_node = tree.vertices[new_id]
        #     plan.insert(0,prev_node)
        
        # plan.append(goal_config)
        # print "time: ", time.time() - startTime
        return draw_plan

            


    def selectNode(self, tree, copt):            
        r = self.r2.random()      
        m_q = -1
        
        while r > m_q:
            target_config = self.planning_env.GenerateRandomConfiguration()
            current_id, current_config, curr_cost, currTotalCost = tree.GetNearestVertex(target_config)            
            cmax = tree.getMaxCost()
            
            if current_id == 0:
                 m_q = 0.5
            else:
                m_q = 1 - ((currTotalCost - copt)/(cmax - copt))
                m_q = min(m_q, tree.prob_floor)            

            r = self.r2.random()
            

        return (current_id, current_config, curr_cost, currTotalCost,target_config)

