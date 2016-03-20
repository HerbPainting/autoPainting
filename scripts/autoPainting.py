#!/usr/bin/env python
"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""

import os
if os.environ.get('ROS_DISTRO', 'hydro')[0] <= 'f':
    import roslib
    roslib.load_manifest('herbpy')

import argparse, herbpy, logging, numpy, openravepy, sys
import math


def ConvertPlanToTrajectory(self, plan):

        # Create a trajectory
        traj = openravepy.RaveCreateTrajectory(self.robot.GetEnv(), 'GenericTrajectory')
        config_spec = self.robot.GetActiveConfigurationSpecification()
        traj.Init(config_spec)

        idx = 0
        for pt in plan:
            traj.Insert(idx, pt)
            idx = idx + 1

        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, self.robot, maxvelmult=1, maxaccelmult=1, hastimestamps=False, plannername='ParabolicTrajectoryRetimer')

        return traj

#class DrawIngObject

class DrawingManager(object):
    def __init__(self,robot,arm,plane,canvasCenter,canvasSize):
        self.arm = arm
        self.arm = arm
        self.robot = robot
        self.plane = plane
        self.canvasCenter = canvasCenter
        self.canvasSize = canvasSize
        dist = math.sqrt(math.pow(plane[0],2) + math.pow(plane[1],2) + math.pow(plane[2],2))
        self.normalVectorUnit = plane/dist
        self.canvasOffset = 0.05
        self.currentCanvasPose = None
    def _NextPoint(self,x,y):
        pass

    def _MoveToInitialPreDraw(self):
        armLocation = self.canvasCenter + self.normalVectorUnit * -self.canvasOffset
        tf = numpy.eye(4)
        tf[0][3] = armLocation[0]
        tf[1][3] = armLocation[1]
        tf[2][3] = armLocation[2]
        self.currentCanvasPose = canvasSize/2.0
        self.arm.PlanToEndEffectorPose(tf,execute=True)
        

    def _MoveAcrossCanvas(self,x,y):
        delta = self.currentCanvasPose - numpy.array([x,y])
        dist = math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2))
        
        
    def _MoveToCanvas(self):
        return self.arm.PlanToEndEffectorOffset(self.normalVectorUnit,self.canvasOffset)

    
    def Draw(self,path):
    
        rName = robot.GetName()
        with prpy.Clone(env) as cloned_env:
            r = prpy.Cloned(robot)
            traj1 = r.right_arm.PlanToEndEffectorOffset([0,1,0],.02)
            num = traj1.GetNumOfWaypoints()
            config = traj1.GetWaypoint(1)
            r.right_arm.SetDOFValues(config)
            traj2 = r.right_arm.PlanToEndEffectorOffset([0,1,0],.02)




        traj2a = prpy.util.CopyTrajectory(traj2,env=env)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='utility script for loading HerbPy')
    parser.add_argument('-s', '--sim', action='store_true',
                        help='simulation mode')
    parser.add_argument('-v', '--viewer', nargs='?', const=True,
                        help='attach a viewer of the specified type')
    parser.add_argument('--robot-xml', type=str,
                        help='robot XML file; defaults to herb_description')
    parser.add_argument('--env-xml', type=str,
                        help='environment XML file; defaults to an empty environment')
    parser.add_argument('-b', '--segway-sim', action='store_true',
                        help='simulate base')
    parser.add_argument('-p', '--perception-sim', action='store_true',
                        help='simulate perception')
    parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
    args = parser.parse_args()

    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    herbpy_args = {'sim':args.sim,
                   'attach_viewer':args.viewer,
                   'robot_xml':args.robot_xml,
                   'env_path':args.env_xml,
                   'segway_sim':args.segway_sim,
                   'perception_sim': args.perception_sim}
    if args.sim and not args.segway_sim:
        herbpy_args['segway_sim'] = args.sim
    
    env, robot = herbpy.initialize(**herbpy_args)

    #Todo make center expressed in meters
    dm = DrawingManager(robot,robot.right_arm,numpy.array([1,0,0]),numpy.array([0.6,0,1]),numpy.array[11.5,9]))
    import IPython
    IPython.embed()
