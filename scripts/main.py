#!/usr/bin/env python

#Main file to be executed
#Sets up planning env and start and goal positions
#Calls Planner
#Calls Visualizer to get visualization of tree

from RRTPlanner import RRTPlanner
from PlanningEnv import PlanningEnv
from HeuristicRRTPlanner import HeuristicRRTPlanner
import numpy
import argparse

def main(planning_env, planner):

	raw_input('Press any key to begin planning')
	world_extents = planning_env.getBoundaryLimits()
	start_config = [[1,1], [19.9,19.9], [19.9,1], [1,19.9]]
	goal_config = [[19.9,19.9], [1,1], [1,19.9], [19.9,1]]
	tree = planner.Plan(start_config, goal_config)
    #TreeViz.visualize(tree)
    
    

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='script for testing planners')
	parser.add_argument('-p', '--planner', type=str, default='rrt',
	                    help='The planner to run (rrt or rrtconnect)')

	args = parser.parse_args()
	planning_env = PlanningEnv()

	if args.planner == 'rrt':
	    planner = RRTPlanner(planning_env,4)
	elif args.planner == 'hrrt':
	    planner = HeuristicRRTPlanner(planning_env)
	else:
	    print 'Unknown planner option: %s' % args.planner
	    exit(0)

	main(planning_env, planner)

	import IPython
	IPython.embed()