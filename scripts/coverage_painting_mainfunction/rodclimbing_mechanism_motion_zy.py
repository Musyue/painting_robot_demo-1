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
from jackup_mechanism_basic_functions import *

def rodclimb_mechanism_motion(target_rotation_angle,target_distance):
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rodmechanism_holding_over_flag = rospy.get_param("/renov_up_level/rodmechanism_holding_over_flag")
        rospy.loginfo("%s is %s", rospy.resolve_name('rodmechanism_holding_over_flag'), rodmechanism_holding_over_flag)
        if rodmechanism_holding_over_flag==1:
            rotation_enable(target_rotation_angle)
            rospy.loginfo("the motion of rotation mechanism in process")
            climb_enable(target_distance)
            rospy.loginfo("the homing of climbing mechanism in process")

            # output tracking errors 
            rotation_joint_line_equation_k=rospy.get_param("/renov_up_level/rotation_joint_line_equation_k")
            rotation_joint_line_equation_b=rospy.get_param("/renov_up_level/rotation_joint_line_equation_b")
            pid_tolerance_error_rotation=rospy.get_param("/renov_up_level/pid_tolerance_error_rotation")
            current_rotation_angle=rospy.get_param("/renov_up_level/rotation_abs_encode")*rotation_joint_line_equation_k+rotation_joint_line_equation_b
            rotation_trackingerror= target_rotation_angle-current_rotation_angle
            rospy.logerr(("%s is %f"%("rotation target angle",target_rotation_angle)))
            rospy.logerr(("%s is %f"%("rotation current angle",current_rotation_angle)))
            rospy.logerr(("%s is %f"%("rotation tracking error",rotation_trackingerror)))

            # output tracking errors 
            pid_tolerance_error_climb=rospy.get_param("/renov_up_level/pid_tolerance_error_climb")
            read_line_l0_encode=rospy.get_param("/renov_up_level/read_line_l0_encode")
            climb_tracking_error=read_line_l0_encode+target_distance-rospy.get_param("/renov_up_level/read_line_encode")
            rospy.logerr(("%s is %f"%("tracking error",climb_tracking_error)))

            if abs(rotation_trackingerror)<=pid_tolerance_error_rotation and abs(climb_tracking_error)<=pid_tolerance_error_climb:
                rotation_disable()
                rospy.loginfo("the motion of rotation mechanism is closed")
                climb_disable()
                rospy.loginfo("the motion of climbing mechanism is closed")
                os.system('rosparam set /renov_up_level/rodmechanism_holding_over_flag 0')
                os.system('rosparam set /renov_up_level/climbingmechanism_climbing_over_flag 1')
                break
        rate.sleep()

class FLEX3DOFROBOTHOME():
    def __init__(self,nodename):
        self.nodename=nodename
    def Init_node(self):
        rospy.init_node(self.nodename)
    
def main():
    nodename="climb_flex_bar_home_node"

    hc3dof=FLEX3DOFROBOTHOME(nodename)

    hc3dof.Init_node()
    target_rotation_angle=0.0 #math.pi/2
    target_distance=0.1
    rodclimb_mechanism_motion(target_rotation_angle,target_distance)    


if __name__=="__main__":
    main()
