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

def climb_motion(target_rotation_angle):
    home_climb_flex_bar = rospy.get_param("/renov_up_level/home_climb_flex_bar")
    rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)

    rotation_joint_line_equation_k=rospy.get_param("/renov_up_level/rotation_joint_line_equation_k")
    rotation_joint_line_equation_b=rospy.get_param("/renov_up_level/rotation_joint_line_equation_b")
    pid_tolerance_error_rotation=rospy.get_param("/renov_up_level/pid_tolerance_error_rotation")

    if home_climb_flex_bar==1:
        # motion of rotation mechanism 
        os.system('rosparam set /renov_up_level/enable_control_rotation 1')
        os.system('rosparam set /renov_up_level/enable_control_rotation 0')
        os.system('rosparam set /renov_up_level/open_rotation_flag 1')
        os.system('rosparam set /renov_up_level/rad_control_rotation '+str(target_rotation_angle))
        rospy.loginfo("the motion of rotation mechanism in process")

        # close the motion of climb mechanism
        current_rotation_angle=rospy.get_param("/renov_up_level/rotation_abs_encode")*rotation_joint_line_equation_k+rotation_joint_line_equation_b
        rotation_trackingerror= target_rotation_angle-current_rotation_angle
        rospy.logerr(("%s is %f"%("rotation target angle",target_rotation_angle)))
        rospy.logerr(("%s is %f"%("rotation current angle",current_rotation_angle)))
        rospy.logerr(("%s is %f"%("rotation tracking error",rotation_trackingerror)))
        if abs(rotation_trackingerror)<=pid_tolerance_error_rotation:
            os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
            os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 0')
            os.system('rosparam set /renov_up_level/open_rotation_flag 0')
            os.system('rosparam set /renov_up_level/enable_control_rotation 2')
            os.system('rosparam set /renov_up_level/enable_control_rotation 0')
            rospy.loginfo("the motion of rotation mechanism is closed")

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
    target_rotation_angle=0.0
    while not rospy.is_shutdown():
        climb_motion(target_rotation_angle)
        rate.sleep()

if __name__=="__main__":
    main()

# home_climb_flex_bar = rospy.get_param("/renov_up_level/home_climb_flex_bar")
# rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
# if home_climb_flex_bar==1:
# homing of rotation mechanism 
# os.system('rosparam set /renov_up_level/enable_control_rotation 1')
# os.system('rosparam set /renov_up_level/enable_control_rotation 0')
# os.system('rosparam set /renov_up_level/open_rotation_flag 1')
# os.system('rosparam set /renov_up_level/rad_control_rotation '+str(rotation_angle))
# rospy.loginfo("the homing of rotation mechanism in process")
# time.sleep(5)
# # close the moiton of rotation mechanism  
# # rotation_distance_tracking_over=rospy.get_param("/renov_up_level/rotation_distance_tracking_over")
# # if rotation_distance_tracking_over==1:
# rotation_tracking_error=rospy.get_param('/renov_up_level/')
#     os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
#     os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 0')
#     os.system('rosparam set /renov_up_level/open_hold_flag 0')
#     os.system('rosparam set /renov_up_level/enable_control_stand_bar 2')
#     os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
#     os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 0')
#     rospy.loginfo("the motion of rotation mechanism is closed")
#     # time.sleep(0.05)