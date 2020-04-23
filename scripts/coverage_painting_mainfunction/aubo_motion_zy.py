#!/usr/bin/env python
# -*- coding: utf_8 -*-
"""
id：1---->stand bar
id:2----->roation
id:3------>Upper and lower climbing pole
"""
import rospy
import time
import os
import math
import numpy as np
from std_msgs.msg import String,Float64,Bool
from sensor_msgs.msg import JointState
class Renovation_operation():
    def __init__(self):
        self.current_joints=[]
        self.default_start_joints=rospy.get_param('/aubo_ros_script/aubo_start_point')
        self.default_end_joints=rospy.get_param("/aubo_startup_ns/aubo_end_point")
        self.aubo_move_track_pub=rospy.Publisher('/aubo_ros_script/movet', String, queue_size=1)
        self.aubo_joints_sub=rospy.Subscriber('/aubo_startup_ns/aubo_joints',JointState,self.obtain_aubo_joints,queue_size=10)
    def group_joints_to_string(self,q_list):
        group_joints=""
        for i in range(len(q_list)):
            group_joints+=str(tuple(q_list[i]))
        return group_joints
    def obtain_aubo_joints(self,msg):
        self.current_joints=msg.position[:]

    def manipulator_motion(self,aubo_q_list,rate):
        aubo_joints=[]
        for i in range(len(aubo_q_list)):
            aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])
        pubstring="movet"+self.default_start_joints+self.group_joint_to_string(aubo_joints)

        while not rospy.is_shutdown():
            climbingmechanism_climbing_over_flag=rospy.get_param("/renov_up_level/climbingmechanism_climbing_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('climbingmechanism_climbing_over_flag'), climbingmechanism_climbing_over_flag)
            if climbingmechanism_climbing_over_flag==1:
                rospy.loginfo("the motion of manipulator renovation is in process")
                aubo_move_track_pub.publish(pubstring)
                os.system("rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 0")  

            start_waypoint_joints=np.array(aubo_q_list[0:6])
            end_waypoint_joints=np.array(aubo_q_list[len(aubo_q_list)-6:len(aubo_q_list)])
            end_path_joints=np.array(self.default_end_joints)
            current_aubo_joints=np.array(self.current_joints)
            renovation_tool_tracking_error_01=np.sum((start_waypoint_joints-current_aubo_joints)**2)
            renovation_tool_tracking_error_02=np.sum((end_waypoint_joints-current_aubo_joints)**2)
            manipulator_operation_tracking_error=np.sum((end_path_joints-current_aubo_joints)**2)

            tolerance_tracking_error=0.01
            if abs(renovation_tool_tracking_error_01)<=tolerance_tracking_error:
                rospy.loginfo("the motion of electric switch is open")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 1')
            else:
                rospy.loginfo("the motion of electric switch is closed")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 0')
            if abs(renovation_tool_tracking_error_02)<=tolerance_tracking_error:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 1')
            else:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 0')

            if abs(manipulator_operation_tracking_error)<=tolerance_tracking_error:
                rospy.loginfo("the motion of manipulator renovation is closed")
                os.system('rosparam set /renov_up_level/manipulator_renovation_over_flag 1')
                break
            rate.sleep()
    def manipulator_motion_simulation(self,aubo_q_list,rate):
        aubo_joints=[]
        for i in range(len(aubo_q_list)):
            aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])
        pubstring="movet"+self.default_start_joints+self.group_joint_to_string(aubo_joints)

        count=1
        while not rospy.is_shutdown():
            climbingmechanism_climbing_over_flag=rospy.get_param("/renov_up_level/climbingmechanism_climbing_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('climbingmechanism_climbing_over_flag'), climbingmechanism_climbing_over_flag)
            if climbingmechanism_climbing_over_flag==1:
                rospy.loginfo("the motion of manipulator renovation is in process")
                aubo_move_track_pub.publish(pubstring)
                os.system("rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 0")  

            start_waypoint_joints=np.array(aubo_q_list[0:6])
            end_waypoint_joints=np.array(aubo_q_list[len(aubo_q_list)-6:len(aubo_q_list)])
            end_path_joints=np.array(self.default_end_joints)

            if count==1:
                current_aubo_joints=start_waypoint_joints
                time.sleep(2)
            elif count==2:
                current_aubo_joints=end_waypoint_joints
                time.sleep(2)
            elif count==3:
                current_aubo_joints=end_path_joints
                time.sleep(2)
            count=count+1
            renovation_tool_tracking_error_01=np.sum((start_waypoint_joints-current_aubo_joints)**2)
            renovation_tool_tracking_error_02=np.sum((end_waypoint_joints-current_aubo_joints)**2)
            manipulator_operation_tracking_error=np.sum((end_path_joints-current_aubo_joints)**2)

            tolerance_tracking_error=0.01
            if abs(renovation_tool_tracking_error_01)<=tolerance_tracking_error:
                rospy.loginfo("the motion of electric switch is open")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 1')
            else:
                rospy.loginfo("the motion of electric switch is closed")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 0')
            if abs(renovation_tool_tracking_error_02)<=tolerance_tracking_error:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 1')
            else:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 0')

            if abs(manipulator_operation_tracking_error)<=tolerance_tracking_error:
                rospy.loginfo("the motion of manipulator renovation is closed")
                os.system('rosparam set /renov_up_level/manipulator_renovation_over_flag 1')
                break
            rate.sleep()

def main():
    nodename="renovation_operation"
    rospy.init_node(nodename)
    ratet=1
    rate=rospy.Rate(ratet)

    aubo_q_list=[    
        (-0.28525098, -0.53203763, 1.36669062, -1.24286441, -1.85604731, 1.57079633)
        (0.71039368, -0.53203763, 1.36669062, -1.24286441, -0.86040264, 1.5707963)
        (0.71039368, -0.63763321, 1.4856621, -1.01829734, -0.86040264, 1.57079633)
        (-0.28525098, -0.63763321, 1.4856621, -1.01829734, -1.85604731, 1.57079633)
        (-0.28525098, -0.78704025, 1.5382336, -0.8163188, -1.85604731, 1.57079633)
        (0.71039368, -0.78704025, 1.5382336, -0.8163188, -0.86040264, 1.57079633)
        (0.71039368, -0.96986677, 1.52551268, -0.64621321, -0.86040264, 1.57079633)
        (-0.28525098, -0.96986677, 1.52551268, -0.64621321, -1.85604731, 1.57079633)
    (-0.28525098, -1.17565041, 1.44731776, -0.51862448, -1.85604731, 1.57079633)
   (0.71039368, -1.17565041, 1.44731776, -0.51862448, -0.86040264, 1.57079633)
     (0.71039368, -1.39770205, 1.3012984, -0.44259221, -0.86040264, 1.57079633)
    (-0.28525098, -1.39770205, 1.3012984, -0.44259221, -1.85604731, 1.57079633)]



    aubo5=Renovation_operation()
    aubo5.manipulator_motion(aubo_q_list,rate)
    # aubo5.manipulator_motion_simulation(aubo_q_list,rate)
    
if __name__=="__main__":
    main()
