import numpy, operator
from RRTPlanner import RRTTree
from random import random


class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        tree1 = ftree
        tree2 = rtree
        distance = self.planning_env.ComputeDistance(start_config, goal_config)
        
        while distance > epsilon:
           #extend tree1
            r = random()
            if r > self.planning_env.p:
                q_rand = self.planning_env.GenerateRandomConfiguration()
            else:
                q_rand = tree2.vertices[-1]
            cid, q = tree1.GetNearestVertex(q_rand)
            q_new = self.planning_env.Extend(q,q_rand)
            if q_new != None:
                nid = tree1.AddVertex(q_new)
                tree1.AddEdge(cid, nid)
                #self.planning_env.PlotEdge(tree1.vertices[cid], tree1.vertices[nid])
                oid, q_other = tree2.GetNearestVertex(q_new)
                distance = self.planning_env.ComputeDistance(q_new,q_other)
            if distance < epsilon:
                break;
            #Swap the trees
            temptree = tree1
            tree1 = tree2
            tree2 = temptree

        plan.append(start_config)
        if numpy.array_equal(tree1.vertices[0], start_config):
            
            while nid != 0:
                plan.insert(0,tree1.vertices[nid])
                nid = tree1.edges[nid]
                #prev = tree1.vertices[nid]
            while oid != 0:
                plan.append(tree2.vertices[oid]) #Duplication might happen
                oid = tree2.edges[oid]
                #prev = tree2.vertices[oid]
        else:

            while nid != 0:
                plan.append(tree1.vertices[nid])
                nid = tree1.edges[nid]
                #prev = tree1.vertices[nid]
            while oid != 0:
                plan.insert(0,tree2.vertices[oid])
                oid = tree2.edges[oid]
                #prev = tree2.vertices[oid]
            
                

        plan.append(goal_config)

        
        return plan
