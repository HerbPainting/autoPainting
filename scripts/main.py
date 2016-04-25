#!/usr/bin/env python

#Main file to be executed
#Sets up planning env and start and goal positions
#Calls Planner
#Calls Visualizer to get visualization of tree

from RRTPlanner import RRTPlanner
from PlanningEnv import PlanningEnv
from HeuristicRRTPlanner import HeuristicRRTPlanner
from mixedRRTPlanner import mixedRRTPlanner
import numpy
import argparse

def main(planning_env, planner):

	raw_input('Press any key to begin planning')
	world_extents = planning_env.getBoundaryLimits()
	p1 = [world_extents[0][0]+0.1,world_extents[0][1]+0.1]
	p2 = [world_extents[1][0]-0.1,world_extents[1][1]-0.1]
	# start_config = [[0.1,0.1], [14.9,14.9], [14.9,0.1], [0.1,14.9]]
	# goal_config = [[14.9,14.9], [0.1,0.1], [0.1,14.9], [14.9,0.1]]
	start_config = [[p1[0],p1[1]], [p2[0],p2[1]], [p2[0],p1[1]], [p1[0],p2[1]]]
	goal_config = [[p2[0],p2[1]], [p1[0],p1[1]], [p1[0],p2[1]], [p2[0],p1[1]]]
	tree = planner.Plan(start_config, goal_config)
    #TreeViz.visualize(tree)
    
    

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='script for testing planners')
	parser.add_argument('-p', '--planner', type=str, default='rrt',
	                    help='The planner to run (rrt or rrtconnect)')

	args = parser.parse_args()
	planning_env = PlanningEnv()

	if args.planner == 'rrt':
	    planner = mixedRRTPlanner(planning_env,5)
	elif args.planner == 'hrrt':
	    planner = HeuristicRRTPlanner(planning_env)
	else:
	    print 'Unknown planner option: %s' % args.planner
	    exit(0)

	main(planning_env, planner)

	import IPython
	IPython.embed()