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
        self.current_joints=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.default_start_joints=rospy.get_param('/renov_up_level/aubo_start_point')
        self.default_start_joints2=[0.0,0.7156,2.7524,0.47,-1.487,1.57]
        self.default_end_joints=rospy.get_param("/renov_up_level/aubo_end_point")
        self.default_end_joints2=[0.0,0.7156,2.7524,0.47,-1.487,0] # rospy.get_param("/renov_up_level/aubo_end_point")
        self.tolerance_tracking_error=0.01
        self.aubo_move_track_pub=rospy.Publisher('/aubo_ros_script/movet', String, queue_size=1)
        self.aubo_move_joint_pub = rospy.Publisher('/aubo_ros_script/movej', String, queue_size=1)
        self.aubo_move_line_pub = rospy.Publisher('/aubo_ros_script/movel', String, queue_size=1)
        self.aubo_joints_sub=rospy.Subscriber('/renov_up_level/aubo_joints',JointState,self.obtain_aubo_joints,queue_size=10)
    def group_joints_to_string(self,q_list):
        group_joints=""
        for i in range(len(q_list)):
            group_joints+=str(tuple(q_list[i]))
        return group_joints
    def obtain_aubo_joints(self,msg):
        self.current_joints=msg.position[:]

    def manipulator_motion_simulation(self,aubo_q_list,rate):
        aubo_joints=[]
        for i in range(len(aubo_q_list)):
            aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])
        # rospy.loginfo("aubo joints are: %s",aubo_joints)
        pubstring="movet"+self.group_joints_to_string(aubo_joints)+self.default_end_joints+self.default_start_joints
        # rospy.loginfo("the published string is: %s",pubstring)
        count=1
        while not rospy.is_shutdown():
            climbingmechanism_climbing_over_flag=rospy.get_param("/renov_up_level/climbingmechanism_climbing_over_flag")
            # rospy.loginfo("%s is %s", rospy.resolve_name('climbingmechanism_climbing_over_flag'), climbingmechanism_climbing_over_flag)
            if climbingmechanism_climbing_over_flag==1:
                rospy.logerr("step 4: manipulator_renovation_motion is in process")
                # self.aubo_move_track_pub.publish(pubstring)
                os.system("rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 0") 

                renovation_tool_tracking_error_01=0.0
                renovation_tool_tracking_error_02=0.0
                manipulator_operation_tracking_error=0.0
                tolerance_tracking_error=0.01
                if abs(renovation_tool_tracking_error_01)<=tolerance_tracking_error:
                    rospy.logerr("the motion of electric switch is open")
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 1')
                else:
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 0')
                if abs(renovation_tool_tracking_error_02)<=tolerance_tracking_error:
                    rospy.logerr("the motion of electric switch is closed")
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 1')
                else:
                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 0')

                if abs(manipulator_operation_tracking_error)<=tolerance_tracking_error:
                    rospy.logerr("step 4: manipulator_renovation_motion is closed")
                    os.system('rosparam set /renov_up_level/manipulator_renovation_over_flag 1')
                    break
            rate.sleep()


    def manipulator_tracking_error_computation(self,refence_joints):
        current_aubo_joints1=np.array(self.current_joints)
        manipulator_operation_tracking_errorlist=refence_joints-current_aubo_joints1 
        manipulator_operation_tracking_error=math.sqrt(np.sum((manipulator_operation_tracking_errorlist)**2))
        return manipulator_operation_tracking_error

    def manipulator_firstphase_nonrenovation_motion(self,joints,rate):
        start_joints=joints[0:6]
        end_joints=joints[6:12]
        pubstring="movej"+str(start_joints)+str(end_joints)
        while not rospy.is_shutdown():
            climbingmechanism_climbing_over_flag=rospy.get_param("/renov_up_level/climbingmechanism_climbing_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('climbingmechanism_climbing_over_flag'), climbingmechanism_climbing_over_flag)

            if climbingmechanism_climbing_over_flag==1:
                rospy.logerr("step 4: first phase manipulator_nonrenovation_motion is in process")
                self.aubo_move_joint_pub.publish(pubstring)
                os.system("rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 0") 
                time.sleep(0.5)
                manipulator_operation_tracking_error= manipulator_tracking_error_computation(self,start_joints)
                rospy.logerr("manipulator first phase nonrenovation motion starting error is: %s",str(manipulator_operation_tracking_error))   

                "nonrenovation motion triggering condition"
                if manipulator_operation_tracking_error<=self.tolerance_tracking_error:
                    os.system("rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 1") 
                else:
                    os.system("rosparam set /renov_up_level/firstphase_nonrenovation_motion_flag 1")

            current_aubo_joints=np.array(self.current_joints)
            firstphase_nonrenovation_motion_flag=rospy.get_param("/renov_up_level/firstphase_nonrenovation_motion_flag")
            if firstphase_nonrenovation_motion_flag==1:
                manipulator_operation_tracking_error= manipulator_tracking_error_computation(self,end_joints)
                rospy.logerr("manipulator first phase nonrenovation motion stoping error is: %s",str(manipulator_operation_tracking_error))

                "nonrenovation motion termination condition"
                if abs(manipulator_operation_tracking_error)<=self.tolerance_tracking_error:
                    rospy.logerr("step 4: first phase manipulator_nonrenovation_motion is terminated")
                    os.system('rosparam set /renov_up_level/firstphase_nonrenovation_motion_flag 0')
                    os.system('rosparam set /renov_up_level/firstphase_nonrenovation_over_flag 1')

                    break
            rate.sleep()

    def manipulator_secondphase_renovation_motion(self,joints,rate):
        start_joints=joints[0:6]
        end_joints=joints[len(joints)-6:len(joints)]
        pubstring="movet"+self.group_joints_to_string(joints)
        while not rospy.is_shutdown():
            firstphase_nonrenovation_motion_flag=rospy.get_param("/renov_up_level/firstphase_nonrenovation_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('firstphase_nonrenovation_over_flag'), firstphase_nonrenovation_motion_flag)

            if firstphase_nonrenovation_motion_flag==1:
                rospy.logerr("step 4: second phase manipulator renovation motion is in process")
                self.aubo_move_track_pub.publish(pubstring)
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 1')
                rospy.logerr("the motion of electric switch is open")
                os.system("rosparam set /renov_up_level/firstphase_nonrenovation_over_flag 0") 

                time.sleep(0.5)
                manipulator_operation_tracking_error= manipulator_tracking_error_computation(self,start_joints)
                rospy.logerr("manipulator second phase renovation motion starting error is: %s",str(manipulator_operation_tracking_error))   
                electric_switch_painting_open_state=rospy.get_param("/renov_up_level/write_electric_switch_painting_open")

                "nonrenovation motion triggering condition"
                if manipulator_operation_tracking_error<=self.tolerance_tracking_error or electric_switch_painting_open_state==0:
                    os.system("rosparam set /renov_up_level/firstphase_nonrenovation_over_flag 1") 
                else:
                    os.system("rosparam set /renov_up_level/secondphase_renovation_motion_flag 1")

            current_aubo_joints=np.array(self.current_joints)
            secondphase_renovation_motion_flag=rospy.get_param("/renov_up_level/secondphase_renovation_motion_flag")
            if secondphase_renovation_motion_flag==1:
                manipulator_operation_tracking_error= manipulator_tracking_error_computation(self,end_joints)
                rospy.logerr("manipulator second phase renovation motion stoping error is: %s",str(manipulator_operation_tracking_error))

                "nonrenovation motion termination condition"
                if abs(manipulator_operation_tracking_error)<=self.tolerance_tracking_error:
                    rospy.logerr("step 4: first phase manipulator_nonrenovation_motion is terminated")
                    os.system('rosparam set /renov_up_level/secondphase_renovation_motion_flag 0')
                    os.system('rosparam set /renov_up_level/secondphase_renovation_over_flag 1')

                    os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 1')
                    rospy.logerr("the motion of electric switch is closed")

                    break
            rate.sleep()

            renovation_tool_tracking_errorlist_01=start_waypoint_joints-current_aubo_joints
            renovation_tool_tracking_error_01=math.sqrt(np.sum((renovation_tool_tracking_errorlist_01)**2))
            rospy.loginfo("manipulator renovation tracking error1 is: %s",str(renovation_tool_tracking_error_01))

            renovation_tool_tracking_errorlist_02=end_waypoint_joints-current_aubo_joints
            renovation_tool_tracking_error_02=math.sqrt(np.sum((renovation_tool_tracking_errorlist_02)**2))
            rospy.loginfo("manipulator renovation tracking error2 is: %s",str(renovation_tool_tracking_error_02))

            "painting gun will be triggered if manipulator approaches the starting point of renovation path"
            if abs(renovation_tool_tracking_error_01)<=tolerance_tracking_error:
                rospy.logerr("the motion of electric switch is open")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 1')
            else:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_open 0')

            "painting gun will be triggered if manipulator approaches the end point of renovation path"
            if abs(renovation_tool_tracking_error_02)<=tolerance_tracking_error:
                rospy.logerr("the motion of electric switch is closed")
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 1')
            else:
                os.system('rosparam set /renov_up_level/write_electric_switch_painting_close 0')




    def aubo_motion(self,aubo_q_list,rate):
        aubo_joints=[]
        for i in range(len(aubo_q_list)):
            aubo_joints.append(aubo_q_list["aubo_data_num_"+str(i)])
        joints_string1=self.default_start_joints+self.group_joints_to_string(aubo_joints[0:6])
        joints_string2=self.group_joints_to_string(aubo_joints[6:len(aubo_joints)-6])
        joints_string3=self.group_joints_to_string(aubo_joints[len(aubo_joints)-6:len(aubo_joints)])+self.default_end_joints

        self.aubo_nonrenovation_motion(joints_string1)
        
        self.aubo_renovation_motion(joints_string2)
        self.aubo_nonrenovation_motion(joints_string3)

def main():
    nodename="renovation_operation"
    rospy.init_node(nodename)
    ratet=30
    rate=rospy.Rate(ratet)

    aubo_q_list={"aubo_data_num_0": [-0.28525098, -0.53203763, 1.36669062, -1.24286441, -1.85604731, 1.57079633], "aubo_data_num_1": [0.71039368, -0.53203763, 1.36669062, -1.24286441, -0.86040264, 1.5707963], "aubo_data_num_2": [0.71039368, -0.63763321, 1.4856621, -1.01829734, -0.86040264, 1.57079633], "aubo_data_num_3": [-0.28525098, -0.63763321, 1.4856621, -1.01829734, -1.85604731, 1.57079633], "aubo_data_num_4": [-0.28525098, -0.78704025, 1.5382336, -0.8163188, -1.85604731, 1.57079633], "aubo_data_num_5": [0.71039368, -0.78704025, 1.5382336, -0.8163188, -0.86040264, 1.57079633], "aubo_data_num_6": [0.71039368, -0.96986677, 1.52551268, -0.64621321, -0.86040264, 1.57079633]}

    aubo5=Renovation_operation()
    aubo5.manipulator_motion(aubo_q_list,rate)
    # aubo5.manipulator_motion_simulation(aubo_q_list,rate)
    
if __name__=="__main__":
    main()

