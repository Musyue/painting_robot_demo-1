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
from jackup_mechanism_basic_functions import *

def rodmechanism_homing(target_standbar_displacement):
    standbar_homing_process(target_standbar_displacement)
    flexbar_downwardsmotion_process()
    # close the moiton of standbar mechanism  
    target_homing_distance=-target_standbar_displacement
    read_line_encode_bottom = rospy.get_param("/renov_up_level/read_line_encode_bottom")
    read_line_l0_encoder_bottom = rospy.get_param("/renov_up_level/read_line_l0_encode_bottom")
    current_distance=read_line_encode_bottom-read_line_l0_encoder_bottom
    standbar_tracking_error=target_homing_distance-current_distance
    pid_tolerance_error_standbar=rospy.get_param("/renov_up_level/pid_tolerance_error_standbar")
    rospy.logerr("standbar----target Distance---%s",str(target_homing_distance))
    rospy.logerr("standbar----current Distance---%s",str(current_distance))
    rospy.logerr("standbar----Distance_error---%s",str(standbar_tracking_error))
    if abs(standbar_tracking_error)<pid_tolerance_error_standbar:
        standbar_homing_end()
        flexbar_downwardsmotion_end()

def jackup_mechanism_homing():
    target_standbar_displacement=0.00
    target_rotation_angle=0.00
    target_climb_distance=0.00

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        one_mobilebase_operation_over_flag = rospy.get_param("/renov_up_level/one_mobilebase_operation_over_flag")
        rospy.loginfo("%s is %s", rospy.resolve_name('one_mobilebase_operation_over_flag'), one_mobilebase_operation_over_flag)
        if one_mobilebase_operation_over_flag==1:
            rotation_enable(target_rotation_angle)
            climb_enable(target_climb_distance)
            standbar_homing_process(target_standbar_displacement)
            flexbar_downwardsmotion_process()
            rospy.loginfo("the homing of climbing mechanism in process")
            rospy.loginfo("the homing of rotation mechanism in process")

            # output the tracking error
            target_homing_distance=-target_standbar_displacement
            read_line_encode_bottom = rospy.get_param("/renov_up_level/read_line_encode_bottom")
            read_line_l0_encoder_bottom = rospy.get_param("/renov_up_level/read_line_l0_encode_bottom")
            current_distance=read_line_encode_bottom-read_line_l0_encoder_bottom
            standbar_tracking_error=target_homing_distance-current_distance
            pid_tolerance_error_standbar=rospy.get_param("/renov_up_level/pid_tolerance_error_standbar")
            rospy.logerr("standbar----target Distance is:%s",str(target_homing_distance))
            rospy.logerr("standbar----current Distance is:%s",str(current_distance))
            rospy.logerr("standbar----Distance_error is:%s",str(standbar_tracking_error))
            rospy.loginfo("-----------------------------------------------------------")

            # output tracking errors 
            rotation_joint_line_equation_k=rospy.get_param("/renov_up_level/rotation_joint_line_equation_k")
            rotation_joint_line_equation_b=rospy.get_param("/renov_up_level/rotation_joint_line_equation_b")
            pid_tolerance_error_rotation=rospy.get_param("/renov_up_level/pid_tolerance_error_rotation")
            current_rotation_angle=rospy.get_param("/renov_up_level/rotation_abs_encode")*rotation_joint_line_equation_k+rotation_joint_line_equation_b
            rotation_trackingerror= target_rotation_angle-current_rotation_angle
            rospy.logerr(("%s is %f"%("rotation  mechanism target angle",target_rotation_angle)))
            rospy.logerr(("%s is %f"%("rotation mechanism current angle",current_rotation_angle)))
            rospy.logerr(("%s is %f"%("rotation mechanism tracking error",rotation_trackingerror)))
            rospy.loginfo("-----------------------------------------------------------")

            # output tracking errors 
            pid_tolerance_error_climb=rospy.get_param("/renov_up_level/pid_tolerance_error_climb")
            read_line_l0_encode=rospy.get_param("/renov_up_level/read_line_l0_encode")
            current_distance=rospy.get_param("/renov_up_level/read_line_encode")-read_line_l0_encode
            climb_tracking_error=target_climb_distance-current_distance
            rospy.logerr(("%s is %f"%("climb mechanism target distance",target_climb_distance)))
            rospy.logerr(("%s is %f"%("climb mechanism current distance",current_distance)))
            rospy.logerr(("%s is %f"%("climb mechanism tracking error",climb_tracking_error)))
            rospy.loginfo("-----------------------------------------------------------")

            if abs(rotation_trackingerror)<=pid_tolerance_error_rotation and abs(climb_tracking_error)<=pid_tolerance_error_climb and abs(standbar_tracking_error)<pid_tolerance_error_standbar:
                flexbar_downwardsmotion_end()
                rospy.logerr("the motion of flexbar is closed")
                standbar_homing_end()
                rospy.logerr("the motion of standbar is closed")
                rotation_disable()
                rospy.logerr("the motion of rotation mechanism is closed")
                climb_disable()
                rospy.logerr("the motion of climbing mechanism is closed")
                os.system('rosparam set /renov_up_level/one_mobilebase_operation_over_flag 0')
                os.system('rosparam set /renov_up_level/jackup_mechanism_homing_over_flag 1')
                break
        rate.sleep()



class FLEX3DOFROBOTHOME():
    def __init__(self,nodename):
        self.nodename=nodename
    def Init_node(self):
        rospy.init_node(self.nodename)
    
def main():
    nodename="jackup_mechanism_homing"
    hc3dof=FLEX3DOFROBOTHOME(nodename)
    hc3dof.Init_node()
    jackup_mechanism_homing()

if __name__=="__main__":
    main()



