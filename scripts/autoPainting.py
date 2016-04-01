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

import prpy
import prpy.util


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



def getXRotation(radians):
    rx = numpy.eye(4)
    cs = math.cos(radians)
    sn = math.sin(radians) 
    rx[1][1] = cs
    rx[2][2] = cs
    rx[2][1] = sn
    rx[1][2] = -1 * sn
    return rx


def getYRotation(radians):
    ry = numpy.eye(4)
    cs = math.cos(radians)
    sn = math.sin(radians) 
    ry[0][0] = cs
    ry[2][2] = cs
    ry[0][2] = sn
    ry[2][0] = -1 * sn
    return ry


def getZRotation(radians):
    rz = numpy.eye(4)
    cs = math.cos(radians)
    sn = math.sin(radians) 
    rz[0][0] = cs
    rz[1][1] = cs
    rz[0][1] = -1 * sn
    rz[1][0] = sn
    return rz



def getTransformFromPlane(normalUnitVector):
    px = normalUnitVector[0]
    py = normalUnitVector[1]
    pz = normalUnitVector[2]
    return getYRotation(0.1)
    
    

class DrawingManager(object):
    def __init__(self,env,robot,arm,plane,canvasCenter,canvasSize):
        self.arm = arm
        self.env = env

        if robot.right_arm is arm:
            self.armString = "Right"
        else:
            self.armString = "Left"
        self.robot = robot
        self.plane = plane
        self.Realrobot = robot
        self.Realplane = plane
        self.canvasCenter = canvasCenter
        self.canvasSize = canvasSize
        dist = math.sqrt(math.pow(plane[0],2) + math.pow(plane[1],2) + math.pow(plane[2],2))
        self.normalVectorUnit = plane/dist
        self.canvasOffset = 0.05
        self.currentCanvasPose = None

        
        self.tf = getTransformFromPlane(self.normalVectorUnit)

    
    def _PlanToEndEffectorPose(self,tf):
        realtf = numpy.dot(self.robot.GetTransform(),tf)
        return self.arm.PlanToEndEffectorPose(realtf)
            
    def _MoveToInitialPreDraw(self):
        armLocation = self.canvasCenter + self.normalVectorUnit * -self.canvasOffset
        tf = numpy.eye(4)
        tf[0][3] = armLocation[0]
        tf[1][3] = armLocation[1]
        tf[2][3] = armLocation[2]
        self.currentCanvasPose = self.canvasSize/2.0
        tf = numpy.dot(tf,getYRotation(1.7))
        return self._PlanToEndEffectorPose(tf)
        

    def _MoveAcrossCanvas(self,point):
        delta = self.currentCanvasPose - point
        dist = math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2))

       # pt = numpy.array([point[0],point[1],0])
       # newPt = self.canvasCenter * numpy.dot(self.tf,pt)
       # pt = numpy.array([self.currentCanvasPose[0],self.currentCanvasPose[1],0,1])
       # oldPt = self.canvasCenter * numpy.dot(self.tf,pt)
        Size3d = numpy.array([self.canvasSize[0],self.canvasSize[1],0])
        canvasOrigin = numpy.array(self.canvasCenter - Size3d/2.0)
       # pt = numpy.array([point[0],point[1],0])
       # newPt = self.canvasCenter +numpy.dot(self.tf,pt)
       # pt = numpy.array([self.currentCanvasPose[0],self.currentCanvasPose[1],0,1])
       # oldPt = self.canvasCenter * numpy.dot(self.tf,pt)

        realPose = numpy.array([0,self.currentCanvasPose[0],self.currentCanvasPose[1]])
        realNew = numpy.array([0,point[0],point[1]])
        oldPt = realPose + canvasOrigin
        newPt = realNew + canvasOrigin
        vector = oldPt - newPt

        vector = numpy.array([vector[0],vector[1],vector[2]])
        
        self.currentCanvasPose = point
        print vector
        print dist

        normalizedVector = vector / math.sqrt(math.pow(vector[0],2) + math.pow(vector[1],2) + math.pow(vector[2],2))

        print normalizedVector
        print normalizedVector
        print dist
        #import IPython
        #IPython.embed()
        return self.arm.PlanToEndEffectorOffset(normalizedVector,dist)


        
        
    def _MoveToCanvas(self):
        return self.arm.PlanToEndEffectorOffset(self.normalVectorUnit,self.canvasOffset)

    def _MoveAwayFromCanvas(self):
        return self.arm.PlanToEndEffectorOffset(self.normalVectorUnit,self.canvasOffset)

    
    def Draw(self,points):
     


        #Move to the offset center of the plane
        initalPreDrawPath = self._MoveToInitialPreDraw()        
        robot.ExecutePath(initalPreDrawPath)

        #Move to the initial point of the path
        initial = points[0]
        robot.ExecutePath(self._MoveAcrossCanvas(initial))
    
        #Save real arms
        realRobot = self.robot
        realArm = self.arm

        trajs = []
        with prpy.Clone(env) as cloned_env:
            
            self.robot = prpy.Cloned(robot)
            if self.armString == "Right":
                self.arm = self.robot.right_arm
            else:
                self.arm = self.robot.left_arm

            trajs.append(self._MoveToCanvas())
            self.arm.SetDOFValues(trajs[-1].GetWaypoint(trajs[-1].GetNumWaypoints()-1))
            for x in xrange(len(points) -1):
                index = x + 1
                print points
                print points[index]
                print index
                trajs.append(self._MoveAcrossCanvas(points[index]))
                self.arm.SetDOFValues(trajs[-1].GetWaypoint(trajs[-1].GetNumWaypoints()-1))


            #trajs.append(self._MoveAwayFromCanvas())


        totalTraj = trajs[0]

        idx = totalTraj.GetNumWaypoints()
        for x in xrange(len(trajs)-1):
            index = x + 1
            for ptIndex in xrange(trajs[index].GetNumWaypoints()):
                totalTraj.Insert(idx,trajs[index].GetWaypoint(ptIndex))
                idx = idx + 1
            

        #Reset
        self.robot = realRobot
        self.arm = realArm        

        performTraj = prpy.util.CopyTrajectory(totalTraj,env=robot.GetEnv())


        origLimits = self.arm.GetVelocityLimits()
        limits = origLimits/2
        self.arm.SetVelocityLimits(limits,self.arm.GetIndices())

        robot.ExecutePath(performTraj)
	
        self.arm.SetVelocityLimits(origLimits,self.arm.GetIndices())

        
        #traj2a = prpy.util.CopyTrajectory(traj2,env=env)
     
#            rName = robot.GetName()
#            with prpy.Clone(env) as cloned_env:
#                r = prpy.Cloned(robot)
#                traj1 = r.right_arm.PlanToEndEffectorOffset([0,1,0],.02)
#                num = traj1.GetNumOfWaypoints()
#                config = traj1.GetWaypoint(num-1)
#                r.right_arm.SetDOFValues(config)
#                traj2 = r.right_arm.PlanToEndEffectorOffset([0,1,0],.02)
#


        #trajs.append(self._MoveAwayFromCanvas())

        #traj2a = prpy.util.CopyTrajectory(traj2,env=env)


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

    #Add in canvas cube
    from openravepy import *
    body1 = RaveCreateKinBody(env,'')
    body1.SetName('canvas')
    body1.InitFromBoxes(numpy.array([[0,0,0,0.01,0.2,0.3]]),True) # [x,y,z,size_x,size_y,size_z]  canvas
    env.AddKinBody(body1)
    body2 = RaveCreateKinBody(env,'')
    body2.SetName('Brush')
    body2.InitFromBoxes(numpy.array([[0,0,0,0.01,0.01,0.1]]),True) #[xb,y,z,size_x,size_y,size_z]   brush
    env.AddKinBody(body2)
# brush to hand and grab brush
    arm_transform= robot.right_arm.GetTransform()
    Palm_direction= arm_transform[0:3,2]
    Palm_direction= 0.18* Palm_direction
    arm_transform[0:3,3]+= Palm_direction
    body2.SetTransform(arm_transform)
    robot.right_arm.hand.CloseHand()
    robot.right_arm.SetActive()
    robot.Grab(body2)

    for igeom,geom in enumerate(body1.GetLinks()[0].GetGeometries()):
        color = numpy.array([1,0.,0])
        geom.SetDiffuseColor(color)

    for igeom,geom in enumerate(body2.GetLinks()[0].GetGeometries()):
        color = numpy.array([0,0.,1])
        geom.SetDiffuseColor(color)

# place canvas

    robot_transform=robot.GetTransform()
    robot_transform[0:3,3]= [0.90,0,1]
    #robot_transform[0:3,3]= [1.95,0,1]
    body1.SetTransform(robot_transform)

    import trajoptpy  
    cc = trajoptpy.GetCollisionChecker(env)
    cc.ExcludeCollisionPair(body1.GetLinks()[0],body2.GetLinks()[0])



    #Todo make center expressed in meters
    dm = DrawingManager(env,robot,robot.right_arm,numpy.array([1,0,0]),numpy.array([0.6,0,1]),numpy.array([0.2,0.2]))

    pathSquare  = [numpy.array([0.1001,0.1001]),numpy.array([0.1,0.1]),numpy.array([0.2,0.1]),numpy.array([0.2,0.2]), numpy.array([0.1,0.2]),numpy.array([0.1,0.1])]
    import IPython
    IPython.embed()
