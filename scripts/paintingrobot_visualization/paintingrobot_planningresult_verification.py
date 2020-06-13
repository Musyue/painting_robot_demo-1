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

        self.min_climbing_distance=rospy.get_param('min_climbing_distance')
        self.max_climbing_distance=rospy.get_param('max_climbing_distance')

        # self.manipulatorbase2rodmechanism_offsetlength=self.parameterz
        # self.rodmechanism2ground_offsetlength=0.86
        # self.rodmechanism2lineencoder_offsetlength=0.62
        # self.paintinggun_offsetlength1=-0.53
        # self.paintinggun_offsetlength2=0.53
        # self.paintinggun_offsetdistance=0.20

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
    
    def renovation_planningresults_verification(self,planning_source_dict,rate):
        plane_num_count=0
        mobile_base_point_count=0
        climb_base_count_num=0
        visualization_num=1
        while not rospy.is_shutdown():
            rospy.loginfo("execute the %sth plane"%str(plane_num_count+1))
            rospy.loginfo("execute the %sth mobile base point"%str(mobile_base_point_count+1))
            rospy.loginfo("execute the %sth climb base point"%str(climb_base_count_num+1))

            "verification of target rod climbing mechanism base positions"
            climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["climb_num_"+ str(climb_base_count_num)]
            
            print("the heigth is",climb_data[0])
            self.min_climbing_distance=rospy.get_param('min_climbing_distance')
            self.max_climbing_distance=rospy.get_param('max_climbing_distance')
            if climb_data[0]<self.min_climbing_distance:
                rospy.logerr("the planned climbing value is smaller than the minimum range")
            if climb_data[0]>self.max_climbing_distance:
                rospy.logerr("the planned climbing value is larger than the maxmimum range")
            if climb_data[0]<=self.max_climbing_distance and climb_data[0]>=self.min_climbing_distance:
                rospy.loginfo("the planned climbing value is ok!--------------------------")
            climb_base_count_num+=1
            if climb_base_count_num>=len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                mobile_base_point_count+=1
                climb_base_count_num=0

            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                plane_num_count+=1 
                mobile_base_point_count=0

            if plane_num_count>=len(planning_source_dict):
                rospy.loginfo("painting operation of whole room is over")
                break
            # rate.sleep()


if __name__ == "__main__":
    rospy.init_node("paintingrobot_planningresults_verification", anonymous=True)
    ratet=1
    rate = rospy.Rate(ratet)
    
    CUHK_renovationrobot=Renovationrobot_positions_visualization()
    planning_source_dict=CUHK_renovationrobot.renovation_planning_source_dict_generation()
    CUHK_renovationrobot.renovation_planningresults_verification(planning_source_dict,rate)



