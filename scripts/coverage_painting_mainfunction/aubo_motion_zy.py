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
        self.aubo_move_track_pub=rospy.Publisher('/aubo_ros_script/movet', String, queue_size=1)
        self.aubo_joints_sub=rospy.Subscriber('/renov_up_level/aubo_joints',JointState,self.obtain_aubo_joints,queue_size=10)
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
        # rospy.loginfo("aubo joints are: %s",aubo_joints)
        pubstring="movet"+self.default_start_joints+self.group_joints_to_string(aubo_joints)+self.default_end_joints
        # rospy.loginfo("the published string is: %s",pubstring)
        while not rospy.is_shutdown():
            tolerance_tracking_error=0.05
            start_path_joints=np.array(self.default_start_joints2)
            end_path_joints=np.array(self.default_end_joints2)

            # rospy.loginfo("start path joints is %s",str(start_path_joints))
            climbingmechanism_climbing_over_flag=rospy.get_param("/renov_up_level/climbingmechanism_climbing_over_flag")
            rospy.loginfo("%s is %s", rospy.resolve_name('climbingmechanism_climbing_over_flag'), climbingmechanism_climbing_over_flag)
            if climbingmechanism_climbing_over_flag==1:
                rospy.logerr("step 4: manipulator_renovation_motion is in process")
                # time.sleep(1)
                self.aubo_move_track_pub.publish(pubstring)
                os.system("rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 0") 
                time.sleep(0.5)
                current_aubo_joints1=np.array(self.current_joints)
                manipulator_operation_tracking_errorlist_01=end_path_joints-current_aubo_joints1 
                manipulator_operation_tracking_error_01=math.sqrt(np.sum((manipulator_operation_tracking_errorlist_01)**2))
                rospy.logerr("manipulator motion tracking error1 is: %s",str(manipulator_operation_tracking_error_01))                        
                "manipulator motion will be triggered again if the manipulator is not moved "
                if manipulator_operation_tracking_error_01<=tolerance_tracking_error:
                    os.system("rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 1") 
                else:
                    os.system("rosparam set /renov_up_level/manipulator_renovation_begin_flag 1")

            # rospy.loginfo("end path joints is %s",str(end_path_joints))
            start_waypoint_joints=np.array(aubo_joints[0])
            # rospy.loginfo("start_waypoint_joints is: %s"%str(start_waypoint_joints))
            end_waypoint_joints=np.array(aubo_joints[len(aubo_joints)-1])
            current_aubo_joints=np.array(self.current_joints)
            # rospy.loginfo("current_aubo_joints is %s",str(current_aubo_joints))
            # rospy.loginfo("current_aubo_joints number is %s",str(len(current_aubo_joints)))
            manipulator_renovation_begin_flag=rospy.get_param("/renov_up_level/manipulator_renovation_begin_flag")
            manipulator_operation_tracking_error_02=1.0
            if manipulator_renovation_begin_flag==1:
                manipulator_operation_tracking_errorlist_02=end_path_joints-current_aubo_joints
                manipulator_operation_tracking_error_02=math.sqrt(np.sum((manipulator_operation_tracking_errorlist_02)**2))
            rospy.loginfo("manipulator motion tracking error2 is: %s",str(manipulator_operation_tracking_error_02))

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

            "manipulator motion will be stoped if manipulator approaches the end point of manipulator path"
            if abs(manipulator_operation_tracking_error_02)<=tolerance_tracking_error:
                rospy.logerr("step 4: manipulator_renovation_motion is closed")
                os.system('rosparam set /renov_up_level/manipulator_renovation_over_flag 1')
                os.system('rosparam set /renov_up_level/manipulator_renovation_begin_flag 0')
                break
            


            rate.sleep()
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

                # start_waypoint_joints=np.array(aubo_joints[0])
                # end_waypoint_joints=np.array(aubo_joints[len(aubo_joints)-1])
                # end_path_joints=np.array(self.default_end_joints)

                # if count==1:
                #     current_aubo_joints=start_waypoint_joints
                #     time.sleep(2)
                # elif count==2:
                #     current_aubo_joints=end_waypoint_joints
                #     time.sleep(2)
                # elif count==3:
                #     current_aubo_joints=end_path_joints
                #     time.sleep(2)
                # count=count+1
                # renovation_tool_tracking_error_01=np.sum((start_waypoint_joints-current_aubo_joints)**2)
                # renovation_tool_tracking_error_02=np.sum((end_waypoint_joints-current_aubo_joints)**2)
                # manipulator_operation_tracking_error=np.sum((end_path_joints-current_aubo_joints)**2)
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

    # {"aubo_data_num_0": [-0.2852509833270265,-0.5320376301933496, 1.3666906155038931, -1.2428644078925508, -1.856047310121923, 1.5707963267948966], "aubo_data_num_1": [0.042683074585430525, -0.3900527148454458, 1.5819954553448392, -1.1695444833995081, -1.528113252209466, 1.5707963267948966],  "aubo_data_num_2": [0.4329114645583898, -0.3900527148454458, 1.5819954553448392, -1.1695444833995081, -1.1378848622365068, 1.5707963267948966], "aubo_data_num_3": [0.4329114645583898, -0.5125345839294475, 1.7000146179664348, -0.9290434516939108, -1.1378848622365068, 1.5707963267948966], "aubo_data_num_4": [0.042683074585430525, -0.5125345839294475, 1.7000146179664348, -0.9290434516939108, -1.528113252209466, 1.5707963267948966], "aubo_data_num_5": [0.042683074585430525, -0.682448092204365, 1.7531385254559293, -0.706006035929498, -1.528113252209466, 1.5707963267948966], "aubo_data_num_6": [0.4329114645583898, -0.682448092204365, 1.7531385254559293, -0.706006035929498,-1.1378848622365068, 1.5707963267948966], "aubo_data_num_7": [0.4329114645583898, -0.8868053270894904, 1.7402280492913906, -0.514559277208912, -1.1378848622365068, 1.5707963267948966], "aubo_data_num_8": [0.042683074585430525, -0.8868053270894904, 1.7402280492913906, -0.514559277208912, -1.528113252209466, 1.5707963267948966], "aubo_data_num_9": [0.042683074585430525, -1.1107170432424152, 1.6616466375248908, -0.3692289728224871, -1.528113252209466, 1.5707963267948966], "aubo_data_num_10": [0.4329114645583898, -1.1107170432424152, 1.6616466375248908, -0.3692289728224871, -1.1378848622365068, 1.5707963267948966], "aubo_data_num_11": [0.4329114645583898, -1.3435999281703808, 1.5184152633737877, -0.2795774620456246, -1.1378848622365068, 1.5707963267948966], "aubo_data_num_12": [0.042683074585430525, -1.3435999281703808, 1.5184152633737877, -0.2795774620456246, -1.528113252209466, 1.5707963267948966],}