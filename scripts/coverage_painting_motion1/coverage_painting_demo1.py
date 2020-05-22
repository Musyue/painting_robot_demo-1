#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import commands
import os
import sys
from geometry_msgs.msg import PoseStamped,Quaternion
import tf

coverage_planner_path=rospy.get_param("coverage_planner_path")
# sys.path.append("/data/ros/renov_robot_ws/src/painting_robot_demo/scripts"
# sys.path.append("/home/zy/catkin_ws/src/paintingrobot/painting_robot_demo/scripts")
sys.path.append(coverage_planner_path)
from coverage_painting_planning.coverage_planning_offline1 import *

from mobileplatform_motion1 import *
from jackup_mechanism_homing1 import *
from rod_mechanism_holding1 import *
from rodclimbing_mechanism_motion1 import *
from aubo_motion1 import *

class RenovationRobot():
    def __init__(self):
        self.mat_path=rospy.get_param('mat_data_path')#"/data/ros/renov_robot_ws/src/painting_robot_demo/data/data.mat"
        self.parameterx=rospy.get_param('mat_parameterx')#0.430725381079
        self.parametery=rospy.get_param('mat_parametery')#-0.00033063639818
        self.parameterz=rospy.get_param('mat_parameterz')#0.028625
        self.interval=rospy.get_param('mat_interval')#0.10
    def renovation_planning_source_dict_generation(self):
        planning_source_dict={}
        rbmo=Renovation_BIM_Model_Opreating(self.mat_path,self.parameterx,self.parametery,self.parameterz,self.interval)
        planning_source_dict=rbmo.get_mat_data_json1()
        return planning_source_dict
    def renovationrobot_motion(self,planning_source_dict,rate):
        plane_num_count=0
        mobile_base_point_count=0
        climb_base_count_num=0

        while not rospy.is_shutdown():
            "executing mobile platform motion"
            mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]
            renovation_mobileplatform=mobile_platform()
            # renovation_mobileplatform.mobile_platform_motion(mobiledata,rate)
            renovation_mobileplatform.mobile_platform_motion_simulation(mobiledata,rate)
            
            "executing rod mechanism holding operation when mobile platform motion is over"
            # target_standbar_displacement=holding_rod_mechanism_target_standbar_displacement_computation()
            target_standbar_displacement=0.12
            # rod_mechanism_holding(target_standbar_displacement,rate)
            rod_mechanism_holding_simulation(target_standbar_displacement,rate)

            while not rospy.is_shutdown():
                rospy.loginfo("execute the %sth plane"%str(plane_num_count+1))
                rospy.loginfo("execute the %sth mobile base point"%str(mobile_base_point_count+1))
                rospy.loginfo("execute the %sth climb base point"%str(climb_base_count_num+1))

                climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["climb_num_"+ str(climb_base_count_num)]
                climb_distance=climb_data[0]
                climb_rotation_angle=climb_data[1]

                print("climb distance is:",climb_distance)
                print("climb rotation angle is",climb_rotation_angle)
                # if climb_distance<=0.85 and climb_distance>=0.0:

                # "executing climbing motion of rod climbing mechanism when holding operation is over"
                # rodclimb_mechanism_motion(climb_rotation_angle,climb_distance,rate)
                rodclimb_mechanism_motion_simulation(climb_rotation_angle,climb_distance,rate)

                "exectuing painting operation of manipulator when climbing operation is over"
                aubo_q_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_aubo_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+ str(climb_base_count_num)]
                aubo5=Renovation_operation()
                # for i in range(len(aubo_q_list)):
                #     list1=aubo_q_list["aubo_data_num_"+str(i)]
                #     for j in range(len(list1)):
                #         list1[j]=list1[j]*180/pi
                #     print(list1)
                print("the number of aubo_q is:",len(aubo_q_list))
                aubo5.aubo_motion(aubo_q_list,rate)
                # aubo5.manipulator_motion_simulation(aubo_q_list,rate)
                
                
                "termination condition: all climbing base positions are conversed"                
                climb_base_count_num+=1
                # if climb_base_count_num>=1:
                if climb_base_count_num>=len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
                    mobile_base_point_count+=1
                    climb_base_count_num=0
                    os.system('rosparam set /renov_up_level/one_mobilebase_operation_over_flag 1')
                    break
            "executing jackup motion of jackup mechanism when operation on one mobile base is over"
            # jackup_mechanism_homing(rate)
            jackup_mechanism_homing_simulation(rate)

            "exit condition: all renovation surface is operated"
            if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
                plane_num_count+=1 
                mobile_base_point_count=0
                # rospy.loginfo("plane_num_count is: %s",str(plane_num_count))
            # if plane_num_count==1 and mobile_base_point_count==3:
            #     break
            if plane_num_count>=len(planning_source_dict):
                rospy.loginfo("painting operation of whole room is over")
                break
            break
            rate.sleep()

def main():
    rospy.init_node("painting_opreating_node_with_bim_model")
    ratet=1
    rate = rospy.Rate(ratet)
    CUHK_renovationrobot=RenovationRobot()
    planning_source_dict=CUHK_renovationrobot.renovation_planning_source_dict_generation()
    CUHK_renovationrobot.renovationrobot_motion(planning_source_dict,rate)

if __name__ == '__main__':
    main()

    
# only rod climbing motion simulation is modified, the real motion is not yet