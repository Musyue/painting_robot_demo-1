#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
from transfer import *
from aubo_kinematics import *
from Quaternion import *

class Renovationrobot_inverse_kinematics():
    def __init__(self):
        self.parameterx=0.430725381079
        self.parametery=-0.00033063639818
        self.parameterz=0.1
        self.interval=0.10

    def renovationrobot_joints_computation(self,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell):
        # computation of target joints of mobile platform
        theta_z=manipulatorbase_targetpose_onecell[0][5]
        deltax=self.parameterx*cos(theta_z)-self.parametery*sin(theta_z)
        deltay=self.parameterx*sin(theta_z)+self.parametery*cos(theta_z)
        mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax,-(manipulatorbase_targetpose_onecell[0][1]-deltay),theta_z]

        # computation of target joints of rodclimbing_robot
        rodclimbing_robot_targetjoints=[manipulatorbase_targetpose_onecell[0][2]-self.parameterz,0.0]

        # computation of forward paths of manipulator
        manipulatorendeffector_targetpose_onecell_new=np.zeros(manipulatorendeffector_targetpose_onecell.shape)
        for i in range(len(manipulatorendeffector_targetpose_onecell_new)/2):
            if i%2==0:
                for j in range(len(manipulatorendeffector_targetpose_onecell_new[i])):
                    manipulatorendeffector_targetpose_onecell_new[2*i][j]=manipulatorendeffector_targetpose_onecell[2*i][j]
                    manipulatorendeffector_targetpose_onecell_new[2*i+1][j]=manipulatorendeffector_targetpose_onecell[2*i+1][j]

            else:
                for j in range(len(manipulatorendeffector_targetpose_onecell_new[i])):
                    manipulatorendeffector_targetpose_onecell_new[2*i][j]=manipulatorendeffector_targetpose_onecell[2*i+1][j]
                    manipulatorendeffector_targetpose_onecell_new[2*i+1][j]=manipulatorendeffector_targetpose_onecell[2*i][j]

        # computation of inverse joints of manipulator
        interval = 0.10
        aubo_joints_list=np.array([0.7432146906113353, -0.6072259915236845, 1.387201205355398, 1.9944271968790823, 0.8275816361835613, 1.5707963267948966])
        previous_aubo_joints=aubo_joints_list
        for i in range(len(manipulatorendeffector_targetpose_onecell_new)-1):
            p1=np.array([manipulatorendeffector_targetpose_onecell_new[i][0],manipulatorendeffector_targetpose_onecell_new[i][1],manipulatorendeffector_targetpose_onecell_new[i][2]])
            p2=np.array([manipulatorendeffector_targetpose_onecell_new[i+1][0],manipulatorendeffector_targetpose_onecell_new[i+1][1],manipulatorendeffector_targetpose_onecell_new[i+1][2]])
            distance=sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
            p=np.zeros(3)
            if i==len(manipulatorendeffector_targetpose_onecell_new)-2:
                num = int(distance / self.interval)+2
            else:
                num = int(distance / self.interval)+1
            for j in range(num):
                p[0] = p1[0] + (p2[0] - p1[0]) / distance * self.interval * j
                p[1] = p1[1] + (p2[1] - p1[1]) / distance * self.interval * j
                p[2] = p1[2] + (p2[2] - p1[2]) / distance * self.interval * j
                q=np.array([manipulatorendeffector_targetpose_onecell_new[i][3],manipulatorendeffector_targetpose_onecell_new[i][4],manipulatorendeffector_targetpose_onecell_new[i][5]])
                T_mat_generation=pose2mat()
                mat=T_mat_generation.mat4x4(p,q)
                mat1=np.ravel(mat)
                mat2=mat1.tolist()
                aubo_arm = Aubo_kinematics()
                aubo_joints_onepoint=aubo_arm.GetInverseResult(mat2,previous_aubo_joints)
                previous_aubo_joints=aubo_joints_onepoint
                aubo_joints_list=np.append(aubo_joints_list,aubo_joints_onepoint,axis=0)
        aubo_targetjoints=aubo_joints_list.reshape(len(aubo_joints_list)/6,6)

        return mobileplatform_targetjoints, rodclimbing_robot_targetjoints,aubo_targetjoints


if __name__ == "__main__":
    mat_path="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_planning/script/planner_result/data.mat"
    data = io.loadmat(mat_path)
    manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
    manipulatorendeffector_targetpose=data['manipulator_endeffector_positions']

    Paintrobot = Renovationrobot_inverse_kinematics()
    # for i in range(len(manipulatorbase_targetpose[0])):
    #     for j in range(len(manipulatorbase_targetpose[0][i][0])):
    #         for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
    for i in range(1):
        for j in range(1):
            for k in range(1):
                manipulatorbase_targetpose_onecell = manipulatorbase_targetpose[0][i][0][j][0][k]
                manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]
                mobileplatform_targetjoints, rodclimbing_robot_targetjoints,aubo_targetjoints = Paintrobot.renovationrobot_joints_computation(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)
                print("mobileplatform_targetjoints=:",mobileplatform_targetjoints)
                print("rodclimbing_robot_targetjoints=:",rodclimbing_robot_targetjoints)
                print("aubo_targetjoints=:",aubo_targetjoints)
