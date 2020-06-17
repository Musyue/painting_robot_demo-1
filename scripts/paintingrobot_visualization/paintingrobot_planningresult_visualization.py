#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
import tf

mat_path=rospy.get_param("/renov_up_level/mat_data_path")
coverage_planner_path=rospy.get_param("/renov_up_level/coverage_planner_path")
sys.path.append(coverage_planner_path)

from paintingrobot_planning.coverage_planning_offline_farubim import *
from paintingrobot_planning.robotic_functions.transfer import *
from paintingrobot_planning.robotic_functions.aubo_kinematics import *
from paintingrobot_planning.robotic_functions.Quaternion import *

from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from waypoints_paths_visualization_functions import *

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

class Renovationrobot_positions_visualization():
    def __init__(self):
        self.mat_path = rospy.get_param('mat_data_path') #"/data/ros/renov_robot_ws/src/painting_robot_demo/data/data.mat"
        self.parameterx = rospy.get_param('mat_parameterx') #0.430725381079
        self.parametery = rospy.get_param('mat_parametery') #-0.00033063639818
        self.parameterz = rospy.get_param('mat_parameterz') #0.028625
        self.interval = rospy.get_param('mat_interval') #0.10
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

        self.paintinggun_offsetlength1=-0.53 #"the first one is higher position "
        self.paintinggun_offsetlength2=0.53 #0.53 #"the second one is lower position"

    def mobile_platform_visualization(self,visualization_num,mobileplatform_targetjoints):
        # visualization of target mobile platform positions and mobile platform path
        maobileplatform_T1 = rotz(mobileplatform_targetjoints[2])
        maobileplatform_T2 = r2t(maobileplatform_T1)
        q0 = quaternion(maobileplatform_T2)
        frame = 'map'
        mobileplatform_targepositions=np.array([mobileplatform_targetjoints[0],mobileplatform_targetjoints[1],0, q0.s, q0.v[0, 0], q0.v[0, 1], q0.v[0, 2]])
        scale1=np.array([0.2,0.2,0.2])
        color1=np.array([1.0,0.0,0.0])
        marker1,visualization_num=targetpositions_visualization(mobileplatform_targepositions, frame, visualization_num, scale1, color1)
        self.marker_pub.publish(marker1)
        # rospy.sleep(0.2)
        return visualization_num

    def manipulatorbase_positions_visualization(self,visualization_num,manipulatorbase_targetpose_onecell):
        maobileplatform_T1 = rotz(manipulatorbase_targetpose_onecell[0][5])
        maobileplatform_T2 = r2t(maobileplatform_T1)
        q0 = quaternion(maobileplatform_T2)
        manipulatorbase_targepositions=np.array([manipulatorbase_targetpose_onecell[0][0],manipulatorbase_targetpose_onecell[0][1],manipulatorbase_targetpose_onecell[0][2], q0.s, q0.v[0, 0], q0.v[0, 1], q0.v[0, 2]])

        frame = 'map'
        scale2 = np.array([0.1, 0.1, 0.25])
        color2 = np.array([0.0, 1.0, 0.0])
        visualization_num=visualization_num+1
        marker1,visualization_num = targetpositions_visualization(manipulatorbase_targepositions, frame, visualization_num, scale2, color2)
        self.marker_pub.publish(marker1)
        # rospy.sleep(0.2)
        return visualization_num

    def target_path_visualization(self,visualization_num,manipulatorendeffector_targetpose_onecell):
        "visualization of planned paths of manipulator"       
        frame='map'
        visualization_num=visualization_num+1
        marker1,visualization_num=path1_visualization(manipulatorendeffector_targetpose_onecell,frame,visualization_num)
        self.marker_pub.publish(marker1)
        return visualization_num

    def positions_visualization(self,visualization_num, paintingrobotendeffector_targetpose_onecell, manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell):
        "visualization of target mobile platform base positions"
        visualization_num=visualization_num+1
        visualization_num=self.mobile_platform_visualization(visualization_num, mobileplatform_targetjoints)
        # print("mobileplatform_targetjoints is:",mobileplatform_targetjoints)

        "visualization of target rod climbing mechanism base positions"
        visualization_num=visualization_num+1
        visualization_num=self.manipulatorbase_positions_visualization(visualization_num, manipulatorbase_targetpose_onecell)

        "visualization of manipulator paths"
        visualization_num=visualization_num+1
        visualization_num=self.target_path_visualization(visualization_num, paintingrobotendeffector_targetpose_onecell)
        return visualization_num
    
    def renovation_planning_source_dict_generation(self):
        planning_source_dict={}
        rbmo=Renovation_BIM_Model_Opreating(self.mat_path,self.parameterx,self.parametery,self.parameterz,self.interval)
        planning_source_dict=rbmo.get_mat_data_json1()
        return planning_source_dict
    
    def renovation_planningresults_visualization(self,planning_source_dict,rate):
        plane_num_count=0
        mobile_base_point_count=0
        climb_base_count_num=0
        visualization_num=1

        data = io.loadmat(self.mat_path)
        manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
        manipulatorendeffector_targetpose=data['manipulator_endeffector_positions_onpath']
        paintingrobotendeffector_targetpose=data['renovation_cells_waypioints_onwaypath']

        Robot_arm = Aubo_kinematics()

        while not rospy.is_shutdown():
            rospy.loginfo("execute the %sth plane"%str(plane_num_count+1))
            rospy.loginfo("execute the %sth mobile base point"%str(mobile_base_point_count+1))
            rospy.loginfo("execute the %sth climb base point"%str(climb_base_count_num+1))

            "visualization of mobile platform positions"
            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]  
            visualization_num=self.mobile_platform_visualization(visualization_num, mobiledata)
            visualization_num=visualization_num+1


            "visualization of manipulator base positions"
            manipulatorbase_targetpose_onecell=[]
            manipulatorbase_targetpose_onecell= manipulatorbase_targetpose[0][plane_num_count][0][mobile_base_point_count][0][climb_base_count_num]
            # manipulatorbase_targetpose_onecell1=manipulatorbase_targetpose_onecell[0][:]
            # if climb_base_count_num==0:
            #     manipulatorbase_targetpose_onecell[0][2]=manipulatorbase_targetpose_onecell[0][2]+self.paintinggun_offsetlength1
            # else:
            #     manipulatorbase_targetpose_onecell[0][2]=manipulatorbase_targetpose_onecell[0][2]+self.paintinggun_offsetlength2
            visualization_num=self.manipulatorbase_positions_visualization(visualization_num, manipulatorbase_targetpose_onecell)
            visualization_num=visualization_num+1

            "visualization of manipulator paths"
            paintingrobotendeffector_targetpose_onecell = paintingrobotendeffector_targetpose[0][plane_num_count][0][mobile_base_point_count][0][climb_base_count_num]
            visualization_num=visualization_num+1
            visualization_num=self.target_path_visualization(visualization_num, paintingrobotendeffector_targetpose_onecell)

            climb_base_count_num+=1
            if climb_base_count_num>=len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                mobile_base_point_count+=1
                climb_base_count_num=0

            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                plane_num_count+=1 
                mobile_base_point_count=0

            if plane_num_count>=len(planning_source_dict):
                plane_num_count=0
                mobile_base_point_count=0
                climb_base_count_num=0
                visualization_num=1
                rospy.loginfo("painting operation of whole room is over")
                # break
            # rate.sleep()



if __name__ == "__main__":
    rospy.init_node("paintingrobot_planningresults_visualization", anonymous=True)
    ratet=1
    rate = rospy.Rate(ratet)
    
    CUHK_renovationrobot=Renovationrobot_positions_visualization()
    planning_source_dict=CUHK_renovationrobot.renovation_planning_source_dict_generation()
    CUHK_renovationrobot.renovation_planningresults_visualization(planning_source_dict,rate)



