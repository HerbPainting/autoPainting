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
from RRTPlanner import RRTPlanner
from PlanningEnv import PlanningEnv


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



def getTransformFromPlane(normalUnitVector,canvasCorner):
    a = normalUnitVector[0]
    b = normalUnitVector[1]
    c = normalUnitVector[2]

    y = ( -b*1-c*0 ) / a * -1
    z = ( -b*0-c*1 ) / a * -1
    #z = ( -a*1-b*0 ) / c
    
    if math.isnan(y):
        tfRy = numpy.eye(4)
    else:
        tfRy = getYRotation(math.atan(y))
    
    if math.isnan(z):
        tfRz = numpy.eye(4)
    else:
        tfRz = getZRotation(math.atan(z))
    tfRx = numpy.eye(4)
    tf = numpy.eye(4)

    tf[0][3] = canvasCorner[0]
    tf[1][3] = canvasCorner[1]
    tf[2][3] = canvasCorner[2]
    
    final = numpy.dot(tf,numpy.dot(numpy.dot(tfRy,tfRz),tfRx))
    return final


class DMConfig(object):
    poses = ["Red","Blue","Yellow","Clean","Canvas","PreColor"]
    def __init__(self,robot,arm):
        self.poses = {}
        for color in DMConfig.poses:
            self.poses[pose] = None
        self.arm = arm

        if robot.right_arm is arm:
            self.armString = "Right"
        else:
            self.armString = "Left"
        self.robot = robot

    def config(self):
        self.robot.SetStiffness(0)
        for x in DMConfig.poses:
            raw_input('Configure ' + x + ' pose')
            self.poses[x] = {}
            self.poses[x]["DOF"] = self.arm.GetDOFValues()
            self.poses[x]["TF"]  = numpy.dot(numpy.linalg.inv(robot.GetTransform()), robot.right_arm.GetTransform())
       
    def save(self,name):
        print "save"
        import pickle
        pickle.dump(self.poses,open(name,"wb"))

class DrawingManager(object):
    def __init__(self,env,robot,arm,plane,canvasCorner,canvasSize,configFileName):
        self.arm = arm
        self.env = env
        
        self.loadConfig(configFileName)

        if robot.right_arm is arm:
            self.armString = "Right"
        else:
            self.armString = "Left"
        self.robot = robot
        self.plane = plane
        self.Realrobot = robot
        self.Realplane = plane
        self.canvasCorner = canvasCorner
        self.canvasSize = canvasSize
        dist = math.sqrt(math.pow(plane[0],2) + math.pow(plane[1],2) + math.pow(plane[2],2))
        self.normalVectorUnit = plane/dist
        self.canvasOffset = 0.05
        self.currentCanvasPose = None

        
        self.tf = getTransformFromPlane(self.normalVectorUnit,canvasCorner)

    def loadConfig(self,configFileName):
        import pickle
        #self.config = pickle.load(open(configFileName,"rb")) 
    
    def _PlanToEndEffectorPose(self,tf):
        realtf = numpy.dot(self.robot.GetTransform(),tf)
        return self.arm.PlanToEndEffectorPose(realtf)
            
    def _MoveToInitialPreDraw(self):
        size = numpy.array([self.canvasSize[0],self.canvasSize[1],0])
        center = self.canvasCorner + size/2.0
        
        normalTf = numpy.eye(4)

        normalTf[0][3] = self.normalVectorUnit[0]
        normalTf[1][3] = self.normalVectorUnit[1]
        normalTf[2][3] = self.normalVectorUnit[2]
        
        rtf = self.robot.GetTransform()
        rtf[0][3] = 0
        rtf[1][3] = 0
        rtf[2][3] = 0
        
        tNorm = numpy.dot(rtf,normalTf)
        newNormalVector = numpy.array([tNorm[0][3],tNorm[1][3],tNorm[2][3]])
        armLocation = center + newNormalVector * -self.canvasOffset
        tf = numpy.eye(4)
        tf[0][3] = armLocation[0]
        tf[1][3] = armLocation[1]
        tf[2][3] = armLocation[2]
        self.currentCanvasPose = numpy.array([0,0])
        tf = numpy.dot(tf,getYRotation(math.pi/2))
        return self._PlanToEndEffectorPose(tf)
    
    def _MoveAlongLine(self,point):
        currentPoint = self.arm.GetTransform()
        newPoint = numpy.dot(self.robot.GetTransform(), point)

        vectorizeTF = lambda a: numpy.array( [ a[0][3],a[1][3],a[2][3] ] )
        newVector = vectorizeTF(newPoint)
        currentVector = vectorizeTF(currentPoint)
        delta = currentVector-newVector
        normalDelta = delta/(math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2) ))
        return self.arm.PlanToEndEffectorOffset(normalDelta,dist)
                  

    def _MoveAcrossCanvas(self,point):
        delta = self.currentCanvasPose - point
        dist = math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2))
        rTf = self.robot.GetTransform()

        currentPointTF=numpy.eye(4)
        currentPointTF[1][3] = self.currentCanvasPose[0]
        currentPointTF[2][3] = self.currentCanvasPose[1]
        
        newPointTF=numpy.eye(4)

        newPointTF[1][3] = point[0]
        newPointTF[2][3] = point[1]

        realCurrentPoint = numpy.dot(numpy.dot(rTf,self.tf),currentPointTF)
        realNewPoint = numpy.dot(numpy.dot(rTf,self.tf),newPointTF)
        
        vectorizeTF = lambda a: numpy.array( [ a[0][3],a[1][3],a[2][3] ] )
        newVector = vectorizeTF(realNewPoint)
        currentVector = vectorizeTF(realCurrentPoint)

        delta = currentVector-newVector
        d = math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2))
        normalDelta = delta/(math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2) ))

        self.currentCanvasPose = point

        return self.arm.PlanToEndEffectorOffset(normalDelta,dist)
        
       # pt = numpy.array([point[0],point[1],0])
       # newPt = self.canvasCenter * numpy.dot(self.tf,pt)
       # pt = numpy.array([self.currentCanvasPose[0],self.currentCanvasPose[1],0,1])
       # oldPt = self.canvasCenter * numpy.dot(self.tf,pt)
       # Size3d = numpy.array([self.canvasSize[0],self.canvasSize[1],0])
       # canvasOrigin = numpy.array(self.canvasCenter - Size3d/2.0)
       # pt = numpy.array([point[0],point[1],0])
       # newPt = self.canvasCenter +numpy.dot(self.tf,pt)
       # pt = numpy.array([self.currentCanvasPose[0],self.currentCanvasPose[1],0,1])
       # oldPt = self.canvasCenter * numpy.dot(self.tf,pt)

       # realPose = numpy.array([0,self.currentCanvasPose[0],self.currentCanvasPose[1]])
       # realNew = numpy.array([0,point[0],point[1]])
       # oldPt = realPose + canvasOrigin
       # newPt = realNew + canvasOrigin
       # vector = oldPt - newPt

       # vector = numpy.array([vector[0],vector[1],vector[2]])
        
       # self.currentCanvasPose = point
       # print vector
       # print dist

       # normalizedVector = vector / math.sqrt(math.pow(vector[0],2) + math.pow(vector[1],2) + math.pow(vector[2],2))

       # print normalizedVector
       # print normalizedVector
       # print dist
        #import IPython
        #IPython.embed()
        #return self.arm.PlanToEndEffectorOffset(normalizedVector,dist)


       # return self.arm.PlanToEndEffectorOffset(normalDelta,dist)
        
        
    def _MoveToCanvas(self):
        normalTf = numpy.eye(4)

        normalTf[0][3] = self.normalVectorUnit[0]
        normalTf[1][3] = self.normalVectorUnit[1]
        normalTf[2][3] = self.normalVectorUnit[2]

        rtf = self.robot.GetTransform()
        rtf[0][3] = 0
        rtf[1][3] = 0
        rtf[2][3] = 0
        tNorm = numpy.dot(rtf,normalTf)
        newNormalVector = numpy.array([tNorm[0][3],tNorm[1][3],tNorm[2][3]])
        return self.arm.PlanToEndEffectorOffset(newNormalVector,self.canvasOffset)

    def _MoveAwayFromCanvas(self):
        normalTf = numpy.eye(4)

        normalTf[0][3] = self.normalVectorUnit[0]
        normalTf[1][3] = self.normalVectorUnit[1]
        normalTf[2][3] = self.normalVectorUnit[2]

        rtf = self.robot.GetTransform()
        rtf[0][3] = 0
        rtf[1][3] = 0
        rtf[2][3] = 0
        tNorm = numpy.dot(rtf,normalTf)
        newNormalVector = numpy.array([-1*tNorm[0][3],-1*tNorm[1][3],-1*tNorm[2][3]])
        return self.arm.PlanToEndEffectorOffset(newNormalVector,self.canvasOffset)


    def GetColor(self,color):

        #MoveToPreColor
        self.arm.PlanToConfiguration(self.config["PreColor"]["DOF"],execute=True)
        #ParrellelMove
        
        self._MoveAlongLine(self.config[color]["TF"])
                
        #Table cost?

        #Dip 

        #UnDip

        #UndoPremove

        #Move To Plane

        #This could be cached
        pass
        #self.arm.PlanToConfiguration
    def CleanBrush(self):
        pass
        

    def Draw(self,points):
        #Move to the offset center of the plane
        initalPreDrawPath = self._MoveToInitialPreDraw()        
        robot.ExecutePath(initalPreDrawPath)

        #Move to the initial point of the path
        initial = points[0]
        robot.ExecutePath(self._MoveAcrossCanvas(initial))
        
        try:
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
                    trajs.append(self._MoveAcrossCanvas(points[index]))
                    self.arm.SetDOFValues(trajs[-1].GetWaypoint(trajs[-1].GetNumWaypoints()-1))


                trajs.append(self._MoveAwayFromCanvas())

            totalTraj = trajs[0]

            idx = totalTraj.GetNumWaypoints()
            for x in xrange(len(trajs)-1):
                index = x + 1
                for ptIndex in xrange(trajs[index].GetNumWaypoints()):
                    totalTraj.Insert(idx,trajs[index].GetWaypoint(ptIndex))
                    idx = idx + 1
        finally:
            #Reset
            self.robot = realRobot
            self.arm = realArm        

        performTraj = prpy.util.CopyTrajectory(totalTraj,env=robot.GetEnv())
        origLimits = self.arm.GetVelocityLimits()
        try:
            #Set the speed limits,  this should be controled by the --sim argument
            limits = origLimits/2
            self.arm.SetVelocityLimits(limits,self.arm.GetIndices())

            robot.ExecutePath(performTraj)
        finally:
            self.arm.SetVelocityLimits(origLimits,self.arm.GetIndices())


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


    rtf = robot.GetTransform()
    rtf[0][3]=5
    rtf[0][1]=4
    robot.SetTransform(rtf)

# Add and place table
    # table = env.ReadKinBodyXMLFile('../data/objects/table.kinbody.xml')
    # env.Add(table)

    # table_pose = numpy.array([[ 0, 0, -1, 1.5], 
    #                           [-1, 0,  0, 0], 
    #                           [ 0, 1,  0, 0], 
    #                           [ 0, 0,  0, 1]])
    # table.SetTransform(table_pose)
    #Add in canvas cube
    from openravepy import *
    body = RaveCreateKinBody(env,'')
    body.SetName('testbody')
    body.InitFromBoxes(numpy.array([[0.6,0,1,0.01,0.2,0.3]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
    env.AddKinBody(body)
    #Add table
    table = env.ReadKinBodyXMLFile('../data/objects/table.kinbody.xml')
    env.Add(table)

    table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                              [-1, 0,  0, 0], 
                              [ 0, 1,  0, 0], 
                              [ 0, 0,  0, 1]])
    table.SetTransform(table_pose)

    #dm = DrawingManager(env,robot,robot.right_arm,numpy.array([1.0,-1,-1]),numpy.array([0.7,0.0,1]),numpy.array([0.2,0.2]))
    dm = DrawingManager(env,robot,robot.right_arm,numpy.array([1.0,0,0]),numpy.array([0.7,0.0,1]),numpy.array([0.2,0.2]),"default.cfg")
    




# RRT planner

    planning_env = PlanningEnv()
    planner = RRTPlanner(planning_env)
 


    def DrawRRT():
       raw_input('Press any key to begin planning')
       world_extents = planning_env.getBoundaryLimits()
       start_config = [[1,1], [3.9,3.9], [3.9,1], [1,3.9]]
       goal_config = [[3.9,3.9], [1,1], [1,3.9], [3.9,1]]
       draw_plan = planner.Plan(start_config, goal_config)
  
       
       for i in xrange(len(draw_plan)):
       	   arr1=numpy.asarray(draw_plan[i][0])
       	   arr2=numpy.asarray(draw_plan[i][1])

           print "STarting debug message"
           print arr1
           print arr2
           
           path_RRT= [arr1,arr2]
           
           print path_RRT
           dm.Draw(path_RRT)
       

      
    pathSquare  = [numpy.array([0.1,0.1]),numpy.array([0.2,0.1]),numpy.array([0.2,0.2]), numpy.array([0.1,0.2]),numpy.array([0.1,0.1])]
    import IPython
    IPython.embed()
