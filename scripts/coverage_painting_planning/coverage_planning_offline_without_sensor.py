#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
from math import *
import numpy as np
import numpy.matlib
# import moveit_commander
import scipy.io as io
from transfer import *
from aubo_kinematics import *
from Quaternion import *
from std_msgs.msg import String,Float64,Bool
from sensor_msgs.msg import JointState
import json
class Renovation_BIM_Model_Opreating():
    def __init__(self,mat_path,parameterx,parametery,parameterz,interval):
        self.parameterx=parameterx#0.430725381079
        self.parametery=parametery#-0.00033063639818
        self.parameterz=parameterz#0.028625
        self.interval=  interval#0.10
        self.mat_path=mat_path
    def renovationrobot_joints_computation_1(self,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell):

        # computation of target joints of mobile platform
        theta_z=manipulatorbase_targetpose_onecell[0][5]
        deltax=self.parameterx*cos(theta_z)-self.parametery*sin(theta_z)
        deltay=self.parameterx*sin(theta_z)+self.parametery*cos(theta_z)
        mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax,(manipulatorbase_targetpose_onecell[0][1]-deltay),theta_z]

        # computation of target joints of rodclimbing_robot
        rodclimbing_robot_targetjoints=[manipulatorbase_targetpose_onecell[0][2]-self.parameterz-0.7,0.0]
        # motion of rod climbing robot
        # print("rodclimbing_robot_targetjoints=",rodclimbing_robot_targetjoints)

        # computation of inverse joints of manipulator
        aubo_joints_list=np.array([-0.2852509833270265, -0.5320376301933496, 1.3666906155038931, -1.2428644078925508, -1.856047310121923,1.5707963267948966])
        previous_aubo_joints=aubo_joints_list

        for i in range(len(manipulatorendeffector_targetpose_onecell)):
            p=np.zeros(3)
            p[0]=manipulatorendeffector_targetpose_onecell[i][0]
            p[1]=manipulatorendeffector_targetpose_onecell[i][1]
            p[2]=manipulatorendeffector_targetpose_onecell[i][2]
            q = np.array(
                [manipulatorendeffector_targetpose_onecell[i][3], manipulatorendeffector_targetpose_onecell[i][4],
                 manipulatorendeffector_targetpose_onecell[i][5]])
            T_mat_generation = pose2mat()
            mat = T_mat_generation.mat4x4(p, q)
            mat1 = np.ravel(mat)
            mat2 = mat1.tolist()
            aubo_arm = Aubo_kinematics()
            aubo_joints_onepoint = aubo_arm.GetInverseResult(mat2, previous_aubo_joints)
            previous_aubo_joints = aubo_joints_onepoint
            aubo_joints_list = np.append(aubo_joints_list, aubo_joints_onepoint, axis=0)

        points_num=len(aubo_joints_list)/6
        for i in range(points_num):
            # motion of manipulator
            aubo_joints=np.array(aubo_joints_list[6*i:6*i+6])
            # print('aubo_joints=:',aubo_joints)

        aubo_targetjoints = aubo_joints_list.reshape(len(aubo_joints_list) / 6, 6)
        return mobileplatform_targetjoints, rodclimbing_robot_targetjoints, aubo_targetjoints

    def print_json(self,data):
        print(json.dumps(data, sort_keys=True, indent=4, separators=(', ', ': '), ensure_ascii=False))
    def array_to_dictlist(self,data):
        datadict={}
        for i in range(len(data)):

            datadict.update({("aubo_data_num_"+str(i)):list(data[i])})
        return datadict
    def get_mat_data_json(self):
        data = io.loadmat(self.mat_path)
        manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
        manipulatorendeffector_targetpose=data['manipulator_endeffector_positions']
        mobile_base=[]
        data_result={}
        room_num={}
        plan_num={}
        mobile_way_point={}
        climb_way_point={}
        rotation_way_point={}
        aubo_joint_space_point={}
        mobile_way_point_data={}
        clim_way_temp={}

        for i in range(len(manipulatorbase_targetpose[0])):#plane
            for j in range(len(manipulatorbase_targetpose[0][i][0])):#v
                for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):#h
        
                    manipulatorbase_targetpose_onecell= manipulatorbase_targetpose[0][i][0][j][0][k]
                    manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]
                    mobileplatform_targetjoints, rodclimbing_robot_targetjoints,aubo_targetjoints = self.renovationrobot_joints_computation_1(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)
                    climb_way_point.update({("climb_num_"+str(k)):rodclimbing_robot_targetjoints})
                    aubo_joint_space_point.update({("aubo_planning_voxel_num_"+str(k)):self.array_to_dictlist(aubo_targetjoints)})

                mobile_base.append(mobileplatform_targetjoints)
                mobile_way_point_data.update({("mobile_data_num_"+str(j)):mobileplatform_targetjoints})
                mobile_way_point.update({("moible_way_num_"+str(i)):mobile_way_point_data,("current_mobile_way_climb_num_"+str(j)):climb_way_point,("current_mobile_way_aubo_num_"+str(j)):aubo_joint_space_point})
                climb_way_point={}
                aubo_joint_space_point={}
                
            plan_num.update({("plane_num_"+str(i)):mobile_way_point})
            mobile_way_point_data={}
            
        self.print_json(plan_num)
        return plan_num
def main():
    mat_path="/data/ros/yue_wk_2019/src/painting_robot_demo/data/data.mat"
    parameterx=0.430725381079
    parametery=-0.00033063639818
    parameterz=0.028625
    interval=0.10
    data = io.loadmat(mat_path)
    # print(data) 
    manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
    # print(len(manipulatorbase_targetpose[0]))
    manipulatorendeffector_targetpose=data['manipulator_endeffector_positions']
    mobile_base=[]
    Paintrobot = Renovation_BIM_Model_Opreating(mat_path,parameterx,parametery,parameterz,interval)


    data_result={}
    room_num={}
    plan_num={}
    mobile_way_point={}
    climb_way_point={}
    rotation_way_point={}
    aubo_joint_space_point={}
    # count_mobile_num=0
    mobile_way_point_data={}
    clim_way_temp={}
    for i in range(len(manipulatorbase_targetpose[0])):#plane
        # print("plane",i)
        for j in range(len(manipulatorbase_targetpose[0][i][0])):#v
            # print("v num",j)

            for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):#h
                
                manipulatorbase_targetpose_onecell= manipulatorbase_targetpose[0][i][0][j][0][k]
                manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]

                mobileplatform_targetjoints, rodclimbing_robot_targetjoints,aubo_targetjoints = Paintrobot.renovationrobot_joints_computation_1(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)
                climb_way_point.update({("climb_num_"+str(k)):rodclimbing_robot_targetjoints})
                aubo_joint_space_point.update({("aubo_planning_voxel_num_"+str(k)):Paintrobot.array_to_dictlist(aubo_targetjoints)})

            mobile_base.append(mobileplatform_targetjoints)
            mobile_way_point_data.update({("mobile_data_num_"+str(j)):mobileplatform_targetjoints})
            mobile_way_point.update({("current_mobile_way_climb_num_"+str(j)):climb_way_point,("current_mobile_way_aubo_num_"+str(j)):aubo_joint_space_point})    
        mobile_way_point.update({("moible_way_num_"+str(i)):mobile_way_point_data})
        climb_way_point={}
        aubo_joint_space_point={}
            
        
        plan_num.update({("plane_num_"+str(i)):mobile_way_point})
        mobile_way_point_data={}
        mobile_way_point={}

            
    Paintrobot.print_json(plan_num)
if __name__ == "__main__":
    main()
