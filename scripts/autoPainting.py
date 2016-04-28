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
from mixedRRTPlanner import mixedRRTPlanner
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
    z = ( -b*0-c*1 ) / a 
    #z = ( -a*1-b*0 ) / c
    
    if math.isnan(y):
        tfRy = numpy.eye(4)
    else:
        tfRy = getZRotation(math.atan(y))
    
    if math.isnan(z):
        tfRz = numpy.eye(4)
    else:
        tfRz = getYRotation(math.atan(z))
    tfRx = numpy.eye(4)
    tf = numpy.eye(4)

    tf[0][3] = canvasCorner[0]
    tf[1][3] = canvasCorner[1]
    tf[2][3] = canvasCorner[2]
    
    final = numpy.dot(tf,numpy.dot(numpy.dot(tfRy,tfRz),tfRx))
    return final


def planeFromPoint(a,b,c):
    l1 = a - b
    l2 = a - c

    normal = numpy.cross(l1,l2)

class DMConfig(object):
    poses = ["Red","Blue","Yellow","Clean","PreColor","Dab","Canvas"]
    def __init__(self,robot,arm,hand):
        self.poses = {}
        for color in DMConfig.poses:
            self.poses[color] = None
        self.arm = arm
        self.hand = hand

        if robot.right_arm is arm:
            self.armString = "Right"
        else:
            self.armString = "Left"
        self.robot = robot

    def _configureCanvasLocation(self):
        self.robot.SetStiffness(0)
        for x in ["Canvas"]:
            raw_input('Configure ' + x + ' pose')
            self.poses[x] = {}
            self.poses[x]["DOF"] = self.arm.GetDOFValues()
            self.poses[x]["TF"]  = numpy.dot(numpy.linalg.inv(robot.GetTransform()), robot.right_arm.GetTransform())
            self.poses[x]["HandDOF"] = self.hand.GetDOFValues()
            self.poses[x]["HandTF"]  = numpy.dot(numpy.linalg.inv(robot.GetTransform()), robot.right_hand.GetTransform())

    def _configureColorLocation(self):
        self.robot.SetStiffness(0)
        for x in DMConfig.poses:
            raw_input('Configure ' + x + ' pose')
            self.poses[x] = {}
            self.poses[x]["DOF"] = self.arm.GetDOFValues()
            self.poses[x]["TF"]  = numpy.dot(numpy.linalg.inv(robot.GetTransform()), robot.right_arm.GetTransform())
            self.poses[x]["HandDOF"] = self.hand.GetDOFValues()
            self.poses[x]["HandTF"]  = numpy.dot(numpy.linalg.inv(robot.GetTransform()), robot.right_hand.GetTransform())
    def config(self):
        self._configureColorLocation()
        self._configurePlane()       

    def _adjustAndGetPoint(self):
        tf = self.arm.GetTransform()

        tf = numpy.dot(numpy.linalg.inv(self.robot.GetTransform()),tf)
        newtf = numpy.eye(4)
        newtf[0][3] = tf[0][3]
        newtf[1][3] = tf[1][3]
        newtf[2][3] = tf[2][3]

        newtf = numpy.dot(newtf,getYRotation(math.pi/2))

        self.robot.SetStiffness(1)
        
        realtf = numpy.dot(self.robot.GetTransform(),newtf)
        self.arm.PlanToEndEffectorPose(realtf,execute=True)
        
        inputValue = "A"
        while(inputValue not in "rR"):
            print "R or r to exit"
            inputValue = raw_input("A = left, D = right, S = down, W = up, Z = in, X = out")
            vector = numpy.array([0,0,0])
            if inputValue in "Aa":
                vector[1] = 1
            elif inputValue in "Dd":
                vector[1] = -1
            elif inputValue in "Ss":
                vector[2] = -1
            elif inputValue in "Ww":
                vector[2] = 1
            elif inputValue in "Zz":
                vector[0] = -1
            elif inputValue in "Xx":
                vector[0] = 1
            elif inputValue in "Rr":
                break
            else:
                inputValue = "Z"
                continue
            vector = vector 

            tf = self.arm.GetTransform()

            tf = numpy.dot(numpy.linalg.inv(self.robot.GetTransform()),tf)
            tf[0][3] += vector[0]
            tf[1][3] += vector[1]
            tf[2][3] += vector[2]

            realtf = numpy.dot(self.robot.GetTransform(),tf)


            tf = self.arm.GetTransform()
            vectorizeTF = lambda a: numpy.array( [ a[0][3],a[1][3],a[2][3] ] )

            cpt = vectorizeTF(tf)
            realpt = vectorizeTF(realtf)

            delta = realpt - cpt
            normalDelta = delta/(math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2) ))
            self.arm.PlanToEndEffectorOffset(normalDelta,0.005,execute=True)
        tf = self.arm.GetTransform()

        tf = numpy.dot(numpy.linalg.inv(self.robot.GetTransform()),tf)
        return tf          

    def _configurePlane(self):
        self.robot.SetStiffness(0)
        raw_input('Move to bottom left corner (On click the arm will rotate square)')
        blPtTf = self._adjustAndGetPoint()
        self.robot.SetStiffness(0)
        raw_input('Move to top left corner (On click the arm will rotate square)')
        tlPtTf = self._adjustAndGetPoint()
        self.robot.SetStiffness(0)
        raw_input('Move to bottm right corner (On click the arm will rotate square)')
        brPtTf = self._adjustAndGetPoint()

        blPt = numpy.array([blPtTf[0][3],blPtTf[1][3],blPtTf[2][3]])
        tlPt = numpy.array([tlPtTf[0][3],tlPtTf[1][3],tlPtTf[2][3]])
        brPt = numpy.array([brPtTf[0][3],brPtTf[1][3],brPtTf[2][3]])
        
        vA = blPt - tlPt
        vB = blPt - brPt
        
        height = math.sqrt(math.pow(vA[0],2) + math.pow(vA[1],2) + math.pow(vA[2],2))
        width = math.sqrt(math.pow(vB[0],2) + math.pow(vB[1],2) + math.pow(vB[2],2))
        plane = numpy.cross(vA,vB)
        

	
        dist = math.sqrt(math.pow(plane[0],2) + math.pow(plane[1],2) + math.pow(plane[2],2))

        plane = plane / dist
        
        self.poses["size"]   = numpy.array([width,height])      
        self.poses["plane"]  = plane
        self.poses["corner"] = blPt 
        self.poses["blPt"] = blPt
        self.poses["tlPt"] = tlPt
        self.poses["brPt"] = brPt
        self.poses["vA"] = vA
        self.poses["vB"] = vB
        self.poses["height"] = height
        self.poses["width"] = width
           
    def save(self,name):
        print "save"
        import pickle
        pickle.dump(self.poses,open(name,"wb"))

    def loadConfig(self,configFileName):
        import pickle
        self.poses = pickle.load(open(configFileName,"rb")) 

class DrawingManager(object):
    def __init__(self,env,robot,arm,configFileName):
        self.arm = arm
        self.env = env

        
        self.loadConfig(configFileName)

        if robot.right_arm is arm:
            self.armString = "Right"
        else:
            self.armString = "Left"
        self.robot = robot
        
        #self.config["plane"] = numpy.array([1,0,0])
        #self.config["corner"] = numpy.array([0.65,0,1.2])
        #self.config["size"] = numpy.array([0.2,0.2])

        self.plane = self.config["plane"]
        
        self.Realrobot = robot
        self.Realplane = self.config["plane"]
        self.canvasCorner = self.config["corner"]
        self.canvasSize = self.config["size"]
        dist = math.sqrt(math.pow(self.plane[0],2) + math.pow(self.plane[1],2) + math.pow(self.plane[2],2))
        self.normalVectorUnit = self.plane/dist
        self.canvasOffset = 0.05
        self.currentCanvasPose = None

        
        self.tf = getTransformFromPlane(self.normalVectorUnit,self.canvasCorner)

    def loadConfig(self,configFileName):
        import pickle
        self.config = pickle.load(open(configFileName,"rb")) 
    
    def _PlanToEndEffectorPose(self,tf):
        realtf = numpy.dot(self.robot.GetTransform(),tf)
        return self.arm.PlanToEndEffectorPose(realtf)
            
    def _MoveToInitialPreDraw(self,point):
        size = numpy.array([0,point[0],point[1]])
        center = self.canvasCorner + size
        
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
        armLocation = self.canvasCorner + newNormalVector * -self.canvasOffset
        tf = numpy.eye(4)
        tf[0][3] = armLocation[0]
        tf[1][3] = armLocation[1]
        tf[2][3] = armLocation[2]
        self.currentCanvasPose = numpy.array([0,0])
        tf = numpy.dot(tf,getYRotation(math.pi/2))
        traj = self._MoveAlongLine(tf)
        
        self.arm.SetDOFValues(traj.GetWaypoint(traj.GetNumWaypoints()-1))
        #self.robot.ExecutePath(traj)

        traj = self._PlanToEndEffectorPose(tf)
        return traj
        #traj = self.arm.PlanToEndEffectorOffset([0,0,1],0.01)
        #self.robot.ExecutePath(traj)
        #return self._PlanToEndEffectorPose(tf)
    
    def _MoveAlongLine(self,point):
        currentPoint = self.arm.GetTransform()
        newPoint = numpy.dot(self.robot.GetTransform(), point)

        vectorizeTF = lambda a: numpy.array( [ a[0][3],a[1][3],a[2][3] ] )
        newVector = vectorizeTF(newPoint)
        currentVector = vectorizeTF(currentPoint)
        delta = newVector - currentVector
        normalDelta = delta/(math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2) ))
        dist = (math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2) ))
        return self.arm.PlanToEndEffectorOffset(normalDelta,dist)
                  

    def _MoveAcrossCanvas(self,point):
        print point
        print self.currentCanvasPose
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
        print delta
        d = math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2))
        normalDelta = delta/(math.sqrt(math.pow(delta[0],2) + math.pow(delta[1],2) + math.pow(delta[2],2) ))
        print "Normal Delta"
        print normalDelta

        self.currentCanvasPose = point

        return self.arm.PlanToEndEffectorOffset(normalDelta,dist)
        
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



    def Dip(self):
        traj = self.arm.PlanToEndEffectorOffset([0,0,-1],0.01)
        self.robot.ExecutePath(traj)

    def Spin(self):
        a = self.arm.GetDOFValues()
        b = self.arm.GetDOFValues()

        a[-1] = -2.8
        self.arm.PlanToConfiguration(a,execute=True)

        a[-1] = 2.8
        self.arm.PlanToConfiguration(a,execute=True)

        self.arm.PlanToConfiguration(b,execute=True)

    def UnDip(self):
        traj = self.arm.PlanToEndEffectorOffset([0,0,1],0.01)
        self.robot.ExecutePath(traj)

    def GetColor(self,color):

        #MoveToPreColor
        self.arm.PlanToConfiguration(self.config["PreColor"]["DOF"],execute=True)
        #self.arm.hand.PlanToConfiguration(self.config["PreColor"]["HandDOF"],execute=True)
        #ParrellelMove
        
        self.arm.PlanToConfiguration(self.config[color]["DOF"],execute=True)
        #performTraj = self._MoveAlongLine(self.config[color]["TF"])
         


        #self.robot.ExecutePath(performTraj)
       
        #Table cost?

        self.Dip()
        if color != "Dab":
            self.Spin()
        self.UnDip()


        self.arm.PlanToConfiguration(self.config["PreColor"]["DOF"],execute=True)


        if color != "Dab":
            self.GetColor("Dab")
        #performTraj = self._MoveAlongLine(self.config["PreColor"]["TF"])
        #self.robot.ExecutePath(performTraj)
        #UndoPremove
        #self.arm.PlanToConfiguration
    def CleanBrush(self):
        self.GetColor("Clean")
        self.GetColor("Dab")
        self.GetColor("Dab")
        self.GetColor("Dab")
        

    def Draw(self,points):

        finalPoints = []
        for p in points:
            x,y = p
            finalPoints.append(numpy.array([x,-y]))

        points = finalPoints
            










        initalDOF = []


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


                traj = self.arm.PlanToConfiguration(self.config["Canvas"]["DOF"])
                self.arm.SetDOFValues(traj.GetWaypoint(traj.GetNumWaypoints()-1))

                
                initial = points[0]
                traj = self._MoveToInitialPreDraw(initial)
                self.arm.SetDOFValues(traj.GetWaypoint(traj.GetNumWaypoints()-1))


                #self.currentCanvasPose = initial
                #initalPreDrawPath = self._MoveToInitialPreDraw()        
                #robot.ExecutePath(initalPreDrawPath)

                #Move to the initial point of the pathls
                initial = points[0]
                traj = self._MoveAcrossCanvas(initial)
                initalDOF = traj.GetWaypoint(traj.GetNumWaypoints()-1)


        except:
            print "Planning Draw Path Failed"
            self.robot = realRobot
            self.arm = realArm
            import traceback
            traceback.print_exc()
            return
        finally:
            #Reset
            self.robot = realRobot
            self.arm = realArm



        self.arm.PlanToConfiguration(initalDOF,execute=True)



        #Move to the offset center of the plane
        try:
            
            self.arm.PlanToConfiguration(initalDOF,execute=True)
           
        except:
            import traceback
            traceback.print_exc()
            print "Hello"
            return
        
        
        try:
            #Save real arms
            #realRobot = self.robot
            #realArm = self.arm

            #trajs = []
            #with prpy.Clone(env) as cloned_env:
            
            #    self.robot = prpy.Cloned(robot)
            #    if self.armString == "Right":
            #        self.arm = self.robot.right_arm
            #    else:
            #        self.arm = self.robot.left_arm

                #trajs.append(self._MoveToCanvas())
                #self.arm.SetDOFValues(trajs[-1].GetWaypoint(trajs[-1].GetNumWaypoints()-1))
                #for x in xrange(len(points) -1):
                #    index = x + 1
                #    trajs.append(self._MoveAcrossCanvas(points[index]))
                #    self.arm.SetDOFValues(trajs[-1].GetWaypoint(trajs[-1].GetNumWaypoints()-1))


                #trajs.append(self._MoveAwayFromCanvas())

            #totalTraj = trajs[0]

            #idx = totalTraj.GetNumWaypoints()
            #for x in xrange(len(trajs)-1):
            #    index = x + 1
            #    for ptIndex in xrange(trajs[index].GetNumWaypoints()):
            #        totalTraj.Insert(idx,trajs[index].GetWaypoint(ptIndex))
            #        idx = idx + 1
            origLimits = self.arm.GetVelocityLimits()
            limits = origLimits/6.0
            self.arm.SetVelocityLimits(limits,12)
            traj = self._MoveToCanvas()
            robot.ExecutePath(traj)
            for x in xrange(len(points) -1):
                index = x + 1
                #trajs.append(self._MoveAcrossCanvas(points[index]))
                traj = self._MoveAcrossCanvas(points[index])
                robot.ExecutePath(traj)

            traj = self._MoveAwayFromCanvas()
            robot.ExecutePath(traj)

        #performTraj = prpy.util.CopyTrajectory(totalTraj,env=robot.GetEnv())
        #origLimits = self.arm.GetVelocityLimits()
       
        except:
            print "Actual Draw Had Issue"
        finally:
            self.arm.SetVelocityLimits(origLimits,2)


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
    from openravepy import *
    #Add in canvas cube
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

    #for igeom,geom in enumerate(body1.GetLinks()[0].GetGeometries()):
    #    color = numpy.array([1,0.,0])
    #    geom.SetDiffuseColor(color)

    for igeom,geom in enumerate(body2.GetLinks()[0].GetGeometries()):
        color = numpy.array([0,0.,1])
        geom.SetDiffuseColor(color)

# place canvas

    #robot_transform=robot.GetTransform()
    #robot_transform[0:3,3]= [0.90,0,1]
    #robot_transform[0:3,3]= [1.95,0,1]
    #body1.SetTransform(robot_transform)


    #rtf = robot.GetTransform()
    #rtf[0][3]=5
    #rtf[0][1]=4
    #robot.SetTransform(rtf)

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
    body.InitFromBoxes(numpy.array([[1.9,0,1,0.01,0.2,0.3]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
    env.AddKinBody(body)
    #Add table
    table = env.ReadKinBodyXMLFile('../data/objects/table.kinbody.xml')
    env.Add(table)

    table_pose = numpy.eye(4)

    table_pose = numpy.dot(getXRotation(3.14/2.0),table_pose)
    table_pose[1][3] = -1.0
    table.SetTransform(table_pose)

    #dm = DrawingManager(env,robot,robot.right_arm,numpy.array([1.0,-1,-1]),numpy.array([0.7,0.0,1]),numpy.array([0.2,0.2]))
    dm = DrawingManager(env,robot,robot.right_arm,"FirstFinalStraightOnFlat")
    




# RRT planner

    planning_env = PlanningEnv()
    planner = mixedRRTPlanner(planning_env,5)
 


    def DrawRRT():
       raw_input('Press any key to begin planning')
       world_extents = planning_env.getBoundaryLimits()
       p1 = [world_extents[0][0]+0.1,world_extents[0][1]+0.1]
       p2 = [world_extents[1][0]-0.1,world_extents[1][1]-0.1]

       
       start_config = [[p1[0],p1[1]], [p2[0],p2[1]], [p2[0],p1[1]], [p1[0],p2[1]]]
       goal_config = [[p2[0],p2[1]], [p1[0],p1[1]], [p1[0],p2[1]], [p2[0],p1[1]]]
       draw_plan = planner.Plan(start_config, goal_config)
  
       colors = ["Yellow","Blue","Red"]
       time = 4
       superCount = 0
       for i in xrange(len(draw_plan)):
       	   arr1=numpy.asarray(draw_plan[i][0])
       	   arr2=numpy.asarray(draw_plan[i][1])

           print "STarting debug message"
           print arr1
           print arr2
           
           path_RRT= [arr1,arr2]
           
           print path_RRT
           time = time -1
           dm.Draw(path_RRT)
           
           if time <= 0:     
                superCount = superCount + 1
                dm.CleanBrush()
                dm.GetColor(colors[superCount%3])
                time = 4
                
       
    dmConfig = DMConfig(robot,robot.right_arm,robot.right_hand)
    def HandPre():
        robot.right_hand.MoveHand(f1=1.75,f2=1.75,f3=1.40,spread=3.14/2) 
    def HandClosed():
        robot.right_hand.MoveHand(f1=1.8,f2=1.8,f3=1.45,spread=3.14/2)







    def rect(bx,by,tx,ty):
        return [(bx,by),
                (bx,ty),
                (tx,ty),
                (tx,by),
                (bx,by)]

    def line(bx,by,tx,ty):
        return [(bx,by),
                (tx,ty)]


    def circle(x,y,radius):
        import math
        points = []
        for i in xrange(5):
            rad = i * math.pi/8
            xp = math.sin(rad) * radius
            yp = math.cos(rad) * radius
            points.append((xp,yp))

        finalPoints = []
        points.reverse()
        for point in points:
            finalPoints.append((x+point[0],y+point[1]))
        points.reverse()
        for point in points:
            finalPoints.append((x-point[0],y+point[1]))
        points.reverse()
        for point in points:
            finalPoints.append((x-point[0],y-point[1]))
        points.reverse()
        for point in points:
            finalPoints.append((x+point[0],y-point[1]))
        finalPoints.append(finalPoints[0])
        return finalPoints


    body = rect(0.1,0.1,0.2,0.3)
    tire1 = rect(0.08,0.08,0.1,0.18)
    tire2 = rect(0.22,0.08,0.20,0.18)

    head = rect(0.13,0.36,0.17,0.39)
    neck = rect(0.15,0.3,0.15,0.36)

    arm1a = line(0.1,0.25,0.04,0.3)
    arm1b = line(0.2,0.25,0.26,0.3)

    arm2a = line(0.04,0.3,0.04,0.23)
    arm2b = line(0.26,0.3,0.26,0.23)


    finger1a = line(0.04,0.23,0.03,0.2)
    finger1b = line(0.04,0.23,0.04,0.2)
    finger1c = line(0.04,0.23,0.05,0.2)


    finger2a = line(0.26,0.23,0.25,0.2)
    finger2b = line(0.26,0.23,0.26,0.2)
    finger2c = line(0.26,0.23,0.27,0.2)


    #def plot(ax,ve):
    #    vx,vy = zip(*ve)
    #    ax.plot(vx,vy,'x-',lw=2,color='black')



    #path = Path(verts, codes)

    #fig = plt.figure()
    #ax = fig.add_subplot(111)
    #patch = patches.PathPatch(path, facecolor='none', lw=2)
    #ax.add_patch(patch)

    #xs, ys = zip(*verts)
    #ax.plot(xs, ys, 'x--', lw=2, color='black', ms=10)

    #Herb Drawing
    #plot(ax,body)
    #plot(ax,tire1)
    #plot(ax,tire2)
    #plot(ax,head)
    #plot(ax,neck)
    #plot(ax,arm1a)
    #plot(ax,arm1b)
    #plot(ax,arm2a)
    #plot(ax,arm2b)

    #plot(ax,finger1a)
    #plot(ax,finger1b)
    #plot(ax,finger1c)

    #plot(ax,finger2a)
    #plot(ax,finger2b)
    #plot(ax,finger2c)

    def Scale(points,factor):
        newPoints = []
        for p in points:
            x,y = p
            x = x*factor
            y = y*factor
            newPoints.append((x,y))
        return newPoints

    def Offset(points,xoff,yoff):
        newPoints = []
        for p in points:
            x,y = p
            x = x + xoff
            y = y + yoff
            newPoints.append((x,y))
        return newPoints




    def convertPoints(pts):
        out = []
        for p in pts:
            out.append(numpy.array(p))
        return out

    def Sanitize(points):
        previousPoint = None
        finalPoints = []
        for p in points:
            if previousPoint != None:
                if abs(p[0] - previousPoint[0]) > 0.001 or abs(p[1] - previousPoint[1]) > 0.001 :
                    finalPoints.append(p)
            else:
                finalPoints.append(p)
            previousPoint = p
        return finalPoints

    def DrawHerb():
        Draw = lambda r: dm.Draw(Sanitize(convertPoints(Offset(Scale(r,0.6),0.0,0.01))))
        
        dm.GetColor("Blue")
        Draw(body)
        dm.GetColor("Red")
        Draw(tire1)
        dm.GetColor("Red")
        Draw(tire2)
        dm.GetColor("Yellow")
        Draw(head)
        dm.GetColor("Yellow")
        Draw(neck)
        dm.GetColor("Blue")
        Draw(arm1a + arm2a+finger1b)
        dm.GetColor("Blue")
        Draw(arm1b + arm2b + finger2b)
        #dm.GetColor("Blue")
        #Draw(arm2a)
        #dm.GetColor("Blue")
        #Draw(arm2b)
        #dm.GetColor("Blue")
        #Draw(finger1a)
        #Draw(finger1b)
        #Draw(finger1c)
        
        #dm.GetColor("Blue")
        #Draw(finger2a)
        #Draw(finger2b)
        #Draw(finger2c)




    

    def DrawHerbName():
        #Herb Word
        H1 = line(0.02,0.05,0.02,0.0)
        H2 = line(0.02,0.025,0.04,0.025)
        H3 = line(0.04,0.05,0.04,0.0)

        E1 = circle(0.06,0.05/4.0,0.05/4.0)
        E2 = line(0.06-0.025/2.0,0.025/2.0,0.06+0.025/2.0,0.025/2.0)
        endIndex = int(len(E1)*7/8.0)
        E1=E1[0:endIndex-1]

        R1 = circle(0.09,0.05/4.0-0.003,0.05/4.0)
        startIndex = int(len(R1)*1/8.0)
        endIndex = int(len(R1)*3/8.0)
        R1=R1[startIndex:endIndex]
        x = R1[-1][0]
        R2 = line(x,0.025,x,0.0)

        B1 = circle(0.12,0.05/4.0,0.05/4.0)
        x = B1[int(len(B1)/2.0)][0]
        B2 = line(x,0.05,x,0.00)



        Draw = lambda r: dm.Draw(Sanitize(convertPoints(Offset(Scale(r,1),0.00,.00))))
        #dm.GetColor("Yellow")
        Draw(H1)
        #dm.GetColor("Yellow")
        Draw(H2)
        #dm.GetColor("Yellow")
        Draw(H3)
        #dm.CleanBrush()
        #dm.GetColor("Blue")
        Draw(E1)
        #dm.GetColor("Blue")
        Draw(E2)
        #dm.CleanBrush()
        #dm.GetColor("Red")
        Draw(R1)
        #dm.GetColor("Red")
        Draw(R2)
        #dm.CleanBrush()
        #dm.GetColor("Blue")
        Draw(B1)
        #dm.GetColor("Blue")
        Draw(B2)


    def DrawHerbName2():
        
        Draw = lambda r: dm.Draw(Sanitize(convertPoints(Offset(Scale(r,1),0.00,.06))))
        #Herb Word
        
        H1 = line(0.02,0.05,0.02,0.0)
        H2 = line(0.02,0.025,0.04,0.025)
        H3 = line(0.04,0.05,0.04,0.0)

        dm.GetColor("Blue")
        Draw(H1)
        Draw(H2)
        Draw(H3)

        E1 = line(0.06,0.05,0.06,0.0)
        E2 = line(0.06,0.05,0.08,0.05)
        E3 = line(0.06,0.025,0.08,0.025)
        E4 = line(0.06,0.00,0.08,0.00)
        E2 = line(0.06-0.025/2.0,0.025/2.0,0.06+0.025/2.0,0.025/2.0)

        dm.GetColor("Red")
        Draw(E1)
        Draw(E2)
        Draw(E3)
        Draw(E4)


        R1 = line(0.10,0.05,0.10,0.0)
        R2 = line(0.10,0.05,0.12,0.05*3.0/4.0)
        R3 = line(0.10,0.025,0.12,0.05*3.0/4.0)
        R4 = line(0.10,0.025,0.12,0.00)


        #R1 = circle(0.09,0.05/4.0-0.003,0.05/4.0)
        #startIndex = int(len(R1)*1/8.0)
        #endIndex = int(len(R1)*3/8.0)
        #R1=R1[startIndex:endIndex]

        #x = R1[-1][0]

        #R2 = line(x,0.025,x,0.0)
        dm.GetColor("Yellow")
        Draw(R1)
        Draw(R2)
        Draw(R3)
        Draw(R4)




        B1 = line(0.15,0.05,0.15,0.0)
        B2 = line(0.15,0.05,0.17,0.05*3.0/4.0)
        B3 = line(0.15,0.025,0.17,0.05*3.0/4.0)
        B4 = line(0.15,0.025,0.17,0.05*1.0/4.0)
        B5 = line(0.15,0.0,0.17,0.05*1.0/4.0)

        dm.GetColor("Blue")    
        Draw(B1)
        Draw(B2)
        Draw(B3)
        Draw(B4)
        Draw(B5)


    test = numpy.array([ 0, -1.43298366, -0.87872026,  1.2935832 , -2.41340378,-0.4731085 , -0.11837519])
    test[0] = 0
    robot.left_arm.SetDOFValues(test)



    pathSquare  = [numpy.array([0.1,0.1]),numpy.array([0.2,0.1]),numpy.array([0.2,0.2]), numpy.array([0.1,0.2]),numpy.array([0.1,0.1])]
    import IPython
    IPython.embed()
