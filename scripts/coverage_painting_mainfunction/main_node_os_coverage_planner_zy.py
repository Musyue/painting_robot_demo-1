#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from math import *
from std_msgs.msg import String,Float64,Bool
import time
import commands
import os
from coverage_planning_offline_without_sensor import *
from geometry_msgs.msg import PoseStamped,Quaternion
import tf

from jackup_mechanism_homing_zy import *
from rod_mechanism_holding_zy import *
from rodclimbing_mechanism_motion import *

class PaintingOpreat():
    def __init__(self):
        self.mobile_go_point_pub = rospy.Publisher('/renov_down_mobile/mobile_go_to_point', PoseStamped, queue_size=1)
        self.aubo_move_track_pub = rospy.Publisher('/aubo_ros_script/movet', String, queue_size=1)
    def Init_node(self):
        rospy.init_node("painting_opreating_node_with_bim_model")
    
    def euler_to_quaternion(self,euler_data):
        return tf.transformations.quaternion_from_euler(euler_data[0],euler_data[1],euler_data[2])
    def pub_posestamped(self,frame_id,posedata,euler_data):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = rospy.Time.now()
        p.pose.position.x=posedata[0]
        p.pose.position.y=posedata[1]
        p.pose.position.z=0
        q = self.euler_to_quaternion(euler_data)
        p.pose.orientation = Quaternion(*q)
        self.mobile_go_point_pub.publish(p)
    def group_joint_to_string(self,q_list):
        resdata=""
        for i in range(len(q_list)):
            resdata+=str(tuple(q_list[i]))
        return resdata
 
def main():
    ratet=1
    Aub=PaintingOpreat()
    Aub.Init_node()
    rate = rospy.Rate(ratet)
    planning_source_dict={}
    mat_path=rospy.get_param('mat_data_path')#"/data/ros/renov_robot_ws/src/painting_robot_demo/data/data.mat"
    parameterx=rospy.get_param('mat_parameterx')#0.430725381079
    parametery=rospy.get_param('mat_parametery')#-0.00033063639818
    parameterz=rospy.get_param('mat_parameterz')#0.028625
    interval=rospy.get_param('mat_interval')#0.10
    StartPoint=rospy.get_param('/aubo_startup_ns/aubo_start_point')#0.10
    rbmo=Renovation_BIM_Model_Opreating(mat_path,parameterx,parametery,parameterz,interval)
    planning_source_dict=rbmo.get_mat_data_json()
    
    plane_num_count=0
    mobile_base_point_count=0
    climb_base_count_num=0

    mobile_tracking_over_flag=0
    holding_over_flag=0
    climbing_over_flag=0
    painting_over_flag=0
    jackup_homing_flag=0

    while not rospy.is_shutdown():
        "executing mobile platform motion"
        mobiledata=planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]["mobile_data_num_"+str(mobile_base_point_count)]
        Aub.pub_posestamped("mobile_base_link",[mobiledata[0],mobiledata[1],0],[0,0,mobiledata[2]])

        "executing rod mechanism holding operation when mobile platform motion is over"
        target_standbar_displacement=holding_rod_mechanism_target_standbar_displacement_computation()
        rod_mechanism_holding(target_standbar_displacement)
        
        "executing climbing motion of rod climbing mechanism when holding operation is over"
        climb_data=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]["climb_num_"+ str(climb_base_count_num)]
        climb_distance=climb_data[0]-1.2
        climb_rotation_angle=climb_data[1]
        rodclimb_mechanism_motion(climb_rotation_angle,climb_distance)

        "exectuing painting operation of manipulator when climbing operation is over"
        aubo_q_list=planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_aubo_num_"+str(mobile_base_point_count)]["aubo_planning_voxel_num_"+ str(climb_base_count_num)]
        tempstr3=[]
        for i in range(len(aubo_q_list)):
            tempstr3.append(aubo_q_list["aubo_data_num_"+str(i)])
        rospy.loginfo("-----tempstr3---%s",str(tempstr3))
        pubstring="movet"+StartPoint + Aub.group_joint_to_string(tempstr3)
        Aub.aubo_move_track_pub.publish(pubstring)
            
        "executing jackup motion of jackup mechanism when operation on one mobile base is over"
        jackup_mechanism_homing()
        
        # transition between climb bases, mobile based and planes
        climb_base_count_num+=1
        if climb_base_count_num>=len(planning_source_dict["plane_num_"+str(plane_num_count)]["current_mobile_way_climb_num_"+str(mobile_base_point_count)]):
            mobile_base_point_count+=1
            os.system('rosparam set /renov_up_level/one_mobilebase_operation_over_flag 1')
        if mobile_base_point_count >= len(planning_source_dict["plane_num_"+str(plane_num_count)]["moible_way_num_"+str(plane_num_count)]):
            plane_num_count+=1 
        
        if plane_num_count>=len(planning_source_dict):
            rospy.loginfo("painting operation of whole room is over")
            break

if __name__ == '__main__':
    main()

    
