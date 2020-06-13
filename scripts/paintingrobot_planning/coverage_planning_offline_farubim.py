#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
# import moveit_commander
import scipy.io as io
from std_msgs.msg import String,Float64,Bool
from sensor_msgs.msg import JointState
import json

path=os.getcwd()
sys.path.append(path)
from robotic_functions.transfer import *
from robotic_functions.aubo_kinematics import *
from robotic_functions.Quaternion import *

coverage_planner_path=rospy.get_param("/renov_up_level/coverage_planner_path")
sys.path.append(coverage_planner_path)
print("coverage_planner_path is",coverage_planner_path)

class Renovation_BIM_Model_Opreating():
    def __init__(self,mat_path,parameterx,parametery,parameterz,interval):
        self.parameterx=parameterx #0.430725381079
        self.parametery=parametery #-0.00033063639818
        self.parameterz=parameterz #0.028625
        self.interval=interval #0.10
        self.mat_path=mat_path

        self.manipulatorbase2rodmechanism_offsetlength=self.parameterz
        self.rodmechanism2ground_offsetlength=0.86
        self.rodmechanism2lineencoder_offsetlength=0.62
        self.paintinggun_offsetlength1=-0.53 #"the first one is higher position "
        self.paintinggun_offsetlength2=0.53 #"the second one is lower position"
        self.paintinggun_offsetdistance=0.20

    def renovationrobot_joints_computation_1(self,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell,aubo_joints_list1,offset_length):

        # computation of target joints of mobile platform
        theta_z=manipulatorbase_targetpose_onecell[0][5]
        deltax=self.parameterx*cos(theta_z)-self.parametery*sin(theta_z)
        deltay=self.parameterx*sin(theta_z)+self.parametery*cos(theta_z)
        mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax,(manipulatorbase_targetpose_onecell[0][1]-deltay),theta_z]

        # computation of target joints of rodclimbing_robot
        # aubo_joints_list1=np.array([37.57375715065746, -11.383079576802844, 70.85252590777286, -79.0851925954416, -53.913611719443864, 168.73807653957576-90.0])

        calibration_offsetlength=-(self.manipulatorbase2rodmechanism_offsetlength+self.rodmechanism2ground_offsetlength+self.rodmechanism2lineencoder_offsetlength)
        rodclimbing_robot_targetjoints=[manipulatorbase_targetpose_onecell[0][2]+calibration_offsetlength+offset_length,0.0]

        # computation of inverse joints of manipulator
        for i in range(len(aubo_joints_list1)):
            aubo_joints_list1[i]=aubo_joints_list1[i]*pi/180
        previous_aubo_joints=aubo_joints_list1
        aubo_joints_list=[]
        for i in range(len(manipulatorendeffector_targetpose_onecell)):
            p=np.zeros(3)
            p[0]=manipulatorendeffector_targetpose_onecell[i][0]
            p[1]=manipulatorendeffector_targetpose_onecell[i][1]
            p[2]=manipulatorendeffector_targetpose_onecell[i][2]
            q = np.array(
                [manipulatorendeffector_targetpose_onecell[i][3],manipulatorendeffector_targetpose_onecell[i][4],
                 manipulatorendeffector_targetpose_onecell[i][5]])
            T_mat_generation = pose2mat()
            mat = T_mat_generation.mat4x4(p, q)
            mat1 = np.ravel(mat)
            mat2 = mat1.tolist()
            aubo_arm = Aubo_kinematics()
            aubo_joints_onepoint = aubo_arm.GetInverseResult(mat2, previous_aubo_joints)
            
            # print("aubo_joints_onepoint is:",aubo_joints_onepoint)
            aubo_joints_onepoint1=[]
            for k in range(len(aubo_joints_onepoint)):
                aubo_joints_onepoint1.append(aubo_joints_onepoint[k])
            print("aubo_joints_onepoint1 is:",aubo_joints_onepoint1)
            aubo_joints_list = np.append(aubo_joints_list, aubo_joints_onepoint, axis=0)

        points_num=len(aubo_joints_list)/6
        for i in range(points_num):
            aubo_joints=np.array(aubo_joints_list[6*i:6*i+6])
        aubo_targetjoints = aubo_joints_list.reshape(len(aubo_joints_list) / 6, 6)
        return mobileplatform_targetjoints, rodclimbing_robot_targetjoints, aubo_targetjoints

    def print_json(self,data):
        print(json.dumps(data, sort_keys=True, indent=4, separators=(', ', ': '), ensure_ascii=False))
    def array_to_dictlist(self,data):
        datadict={}
        for i in range(len(data)):
            datadict.update({("aubo_data_num_"+str(i)):list(data[i])})
        return datadict

    def get_mat_data_json1(self):
        data = io.loadmat(self.mat_path)
        manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
        manipulatorendeffector_targetpose=data['manipulator_endeffector_positions_onpath']
        paintingrobotendeffector_targetpose=data['renovation_cells_waypioints_onwaypath']

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

        aubo_joints_list_1=np.array([37.57375715065746, -11.383079576802844, 70.85252590777286, -79.0851925954416, -53.913611719443864, 168.73807653957576-90.0])
        aubo_joints_list_2=np.array([37.57375715065746, -11.383079576802844, 70.85252590777286, -79.0851925954416, -53.913611719443864, 168.73807653957576-270.0])
        
        for i in range(len(manipulatorbase_targetpose[0])):
            for j in range(len(manipulatorbase_targetpose[0][i][0])):
                for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
                    
                    manipulatorbase_targetpose_onecell= manipulatorbase_targetpose[0][i][0][j][0][k]
                    manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]

                    if k==0:
                        offset_length=self.paintinggun_offsetlength1 
                        mobileplatform_targetjoints, rodclimbing_robot_targetjoints,aubo_targetjoints = self.renovationrobot_joints_computation_1(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell,aubo_joints_list_1,offset_length)
                    else:
                        offset_length=self.paintinggun_offsetlength2 
                        mobileplatform_targetjoints, rodclimbing_robot_targetjoints,aubo_targetjoints = self.renovationrobot_joints_computation_1(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell,aubo_joints_list_2,offset_length)

                    climb_way_point.update({("climb_num_"+str(k)):rodclimbing_robot_targetjoints})
                    aubo_joint_space_point.update({("aubo_planning_voxel_num_"+str(k)):self.array_to_dictlist(aubo_targetjoints)})

                mobile_base.append(mobileplatform_targetjoints)
                mobile_way_point_data.update({("mobile_data_num_"+str(j)):mobileplatform_targetjoints})
                mobile_way_point.update({("current_mobile_way_climb_num_"+str(j)):climb_way_point,("current_mobile_way_aubo_num_"+str(j)):aubo_joint_space_point})    
            
            mobile_way_point.update({("moible_way_num_"+str(i)):mobile_way_point_data})
            climb_way_point={}
            aubo_joint_space_point={}     
            plan_num.update({("plane_num_"+str(i)):mobile_way_point})
            mobile_way_point_data={}
            mobile_way_point={}
        
        self.print_json(plan_num)
        return plan_num
                
    def show_mat(self):
        data = io.loadmat(self.mat_path)
        manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
        manipulatorendeffector_targetpose=data['manipulator_endeffector_positions_onpath']
        i=1
        j=1
        k=1
        manipulatorbase_targetpose_onecell= manipulatorbase_targetpose[0][i][0][j][0][k]
        print("manipulatorbase_targetpose_onecell is:",manipulatorbase_targetpose_onecell)
        manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]
        print("manipulatorendeffector_targetpose_onecell is:",manipulatorendeffector_targetpose_onecell)


def main():
    mat_path="/home/zy/catkin_ws/src/paintingrobot/paintingrobot_underusing/painting_robot_demo/matlab/second_scan_data/second_scan_data2.mat"
    parameterx=0.430725381079
    parametery=-0.00033063639818
    parameterz=0.028625
    interval=0.10
    data = io.loadmat(mat_path)
    Paintrobot2 = Renovation_BIM_Model_Opreating(mat_path,parameterx,parametery,parameterz,interval)
    # planning_source_dict=Paintrobot2.get_mat_data_json1()
    Paintrobot2.show_mat()

if __name__ == "__main__":
    main()
