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


def standbar_homing(target_standbar_displacement):
    target_homing_distance=-target_standbar_displacement
    # homing of standbar mechanism
    os.system('rosparam set /renov_up_level/open_hold_flag 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/distance_control_stand_bar '+str(target_homing_distance))
    rospy.loginfo("the homing of standbar mechanism in process")

    # close the moiton of standbar mechanism  
    read_line_encode_bottom = rospy.get_param("/renov_up_level/read_line_encode_bottom")
    read_line_l0_encoder_bottom = rospy.get_param("/renov_up_level/read_line_l0_encode_bottom")
    current_distance=read_line_encode_bottom-read_line_l0_encoder_bottom
    standbar_tracking_error=target_homing_distance-current_distance
    pid_tolerance_error_standbar=rospy.get_param("/renov_up_level/pid_tolerance_error_standbar")
    rospy.logerr("standbar----target Distance---%s",str(target_homing_distance))
    rospy.logerr("standbar----current Distance---%s",str(current_distance))
    rospy.logerr("standbar----Distance_error---%s",str(standbar_tracking_error))
    if abs(standbar_tracking_error)<pid_tolerance_error_standbar:
        os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
        os.system('rosparam set /renov_up_level/hold_distance_tracking_over 0')
        os.system('rosparam set /renov_up_level/open_hold_flag 0')
        os.system('rosparam set /renov_up_level/enable_control_stand_bar 2')
        os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
        os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 0')
        rospy.loginfo("the motion of standbar mechanism is closed")

class FLEX3DOFROBOTHOME():
    def __init__(self,nodename):
        self.nodename=nodename
    def Init_node(self):
        rospy.init_node(self.nodename)
    
def main():
    nodename="climb_flex_bar_home_node"
    hc3dof=FLEX3DOFROBOTHOME(nodename)
    hc3dof.Init_node()
    rate = rospy.Rate(1)
    # target homing distance should be smaller than 0.05: pos upwards,neg downwards 
    target_standbar_displacement=0.0
    
    while not rospy.is_shutdown():
        home_climb_flex_bar = rospy.get_param("/renov_up_level/home_climb_flex_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
        if home_climb_flex_bar==1:
            standbar_homing(target_standbar_displacement)
            rate.sleep()
if __name__=="__main__":
    main()
