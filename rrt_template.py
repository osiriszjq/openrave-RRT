#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy

#### YOUR IMPORTS GO HERE ####
from myrrtfunctions import *
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [joint1_i, joint2_i, joint3_i,...]

    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeActiveDOFTrajectory(traj,robot)#,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

def tuckarms(env,robot):
    with env:
        jointnames = ['torso_lift_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.24,1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);        
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def GetEETransform(robot,activedofvalues=None):
    if activedofvalues != None:
        robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from environment XML file
    env.Load('pr2table.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms and raise torso
    tuckarms(env,robot);
  
    #set start config
    robot.SetActiveManipulator('leftarm')
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    startconfig = [0.5, 1.19, -1.548, 1.557, -1.32, -0.1928]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)
    with env:
        goalconfig = [0.5, 0.33, -1.548, 1.557, -1.32, -0.1928]
        start = time.clock()
            ### YOUR CODE HERE ###
            ### Plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        step = 0.05
        goal_bias = 0.1

        low,high = robot.GetDOFLimits()
        idx = robot.GetJoint('l_forearm_roll_joint').GetDOFIndex()
        idx2 = robot.GetJoint('l_wrist_flex_joint').GetDOFIndex()
        low[idx] = -np.pi
        high[idx] = np.pi
        low[idx2] = -np.pi
        high[idx2] = np.pi
        robot.SetDOFLimits(low,high)
        low,high = robot.GetActiveDOFLimits()
        path_nodes = rrt(env,robot,goalconfig,startconfig,step,goal_bias)
        handles = []
        for config in path_nodes:
            handles.append(env.plot3(points=GetEETransform(robot,config)[0:3,3],pointsize=3,colors=(1,0,0)))
        path_nodes = smooth(env,robot,path_nodes,150,step)
        for config in path_nodes:
            handles.append(env.plot3(points=GetEETransform(robot,config)[0:3,3],pointsize=3,colors=(0,0,1)))
        path = path_nodes #put your final path in this variable
            #### END OF YOUR CODE ###
        end = time.clock()
        print "Time: ", end - start

            # Now that you have computed a path, convert it to an openrave trajectory 
        traj = ConvertPathToTrajectory(robot, path)

        # Execute the trajectory on the robot.
        if traj != None:
            robot.GetController().SetPath(traj)

    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()
