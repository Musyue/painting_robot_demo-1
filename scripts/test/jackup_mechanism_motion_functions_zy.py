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

def flexbar_upwardsmotion_process():
    # upwards motion of flexbar mechanism 
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 1')
    rospy.logerr("upwards motion of flexbar mecahnism")
def flexbar_upwardsmotion_end():
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 0')
    rospy.logerr("downwards motion of flexbar mechanism is closed")
def flexbar_downwardsmotion_process():
    # downwards motion of flexbar mechanism
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 2')
    rospy.logerr("downwards motion of flexbar mechanism")   
def flexbar_downwardsmotion_end():
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 0') 
    rospy.logerr("downwards motion of flexbar mechanism is closed")   

def standbar_homing_process(target_standbar_displacement):
    target_homing_distance=-target_standbar_displacement
    # homing of standbar mechanism
    os.system('rosparam set /renov_up_level/open_hold_flag 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/distance_control_stand_bar '+str(target_homing_distance))
    rospy.loginfo("the homing of standbar mechanism in process")

def standbar_homing_end():
    os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
    os.system('rosparam set /renov_up_level/hold_distance_tracking_over 0')
    os.system('rosparam set /renov_up_level/open_hold_flag 0')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 2')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 0')
    rospy.loginfo("the motion of standbar mechanism is closed")

def rotation_enable(target_rotation_angle):
    # motion of rotation mechanism 
    os.system('rosparam set /renov_up_level/enable_control_rotation 1')
    os.system('rosparam set /renov_up_level/enable_control_rotation 0')
    os.system('rosparam set /renov_up_level/open_rotation_flag 1')
    os.system('rosparam set /renov_up_level/rad_control_rotation '+str(target_rotation_angle))
    rospy.loginfo("the homing of rotation mechanism in process")
def rotation_disable():
    os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
    os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 0')
    os.system('rosparam set /renov_up_level/open_rotation_flag 0')
    os.system('rosparam set /renov_up_level/enable_control_rotation 2')
    # os.system('rosparam set /renov_up_level/enable_control_rotation 0')

def climb_enable(target_distance):
    # the motion command of climb mechanism
    os.system('rosparam set /renov_up_level/enable_climb_control 1')
    os.system('rosparam set /renov_up_level/enable_climb_control 0')
    os.system('rosparam set /renov_up_level/open_climb_flag 1')
    os.system('rosparam set /renov_up_level/distance_climb_control '+str(target_distance))
    rospy.loginfo("the homing of climbing mechanism in process")

def climb_disable():
    os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
    os.system('rosparam set /renov_up_level/climb_distance_tracking_over 0')
    os.system('rosparam set /renov_up_level/open_climb_flag 0')
    os.system('rosparam set /renov_up_level/enable_climb_control 2')
    os.system('rosparam set /renov_up_level/enable_climb_control 0')
    

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

    rotation_enable(target_rotation_angle)
    climb_enable(target_climb_distance)
    standbar_homing_process(target_standbar_displacement)
    flexbar_downwardsmotion_process()
    
    # output the tracking error
    target_homing_distance=-target_standbar_displacement
    read_line_encode_bottom = rospy.get_param("/renov_up_level/read_line_encode_bottom")
    read_line_l0_encoder_bottom = rospy.get_param("/renov_up_level/read_line_l0_encode_bottom")
    current_distance=read_line_encode_bottom-read_line_l0_encoder_bottom
    standbar_tracking_error=target_homing_distance-current_distance
    pid_tolerance_error_standbar=rospy.get_param("/renov_up_level/pid_tolerance_error_standbar")
    rospy.logerr("standbar----target Distance---%s",str(target_homing_distance))
    rospy.logerr("standbar----current Distance---%s",str(current_distance))
    rospy.logerr("standbar----Distance_error---%s",str(standbar_tracking_error))
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


def standbar_motion_process(target_standbar_displacement):
    target_motion_distance=-target_standbar_displacement
    # motion of standbar mechanism
    os.system('rosparam set /renov_up_level/open_hold_flag 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    os.system('rosparam set /renov_up_level/distance_control_stand_bar '+str(target_motion_distance))
    rospy.loginfo("the homing of standbar mechanism in process")

def standbar_motion_end():
    os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
    os.system('rosparam set /renov_up_level/hold_distance_tracking_over 0')
    os.system('rosparam set /renov_up_level/open_hold_flag 0')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 2')
    os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
    rospy.logerr("the motion of standbar mechanism is closed")

def holding_rod_mechanism_target_standbar_displacement_computation():
    light_scan_to_ceil_distance = rospy.get_param("/renov_up_level/light_scan_to_ceil_distance")
    light_scan_to_top_distance = rospy.get_param("/renov_up_level/light_scan_to_top_distance")
    stand_bar_flex_distance = rospy.get_param("/renov_up_level/stand_bar_flex_distance")
    detax=+light_scan_to_ceil_distance-light_scan_to_top_distance-stand_bar_flex_distance
    rospy.loginfo("detax---%s",str(detax))
    if detax<-0.0:
        rospy.logerr("you can not run the system here,the ceil is too low")
        return 0
    else:
        rospy.loginfo("detax is: %s",str(detax))
    extra_compression_value=0.08
    rospy.loginfo("extra_compression_value---%s",str(extra_compression_value))
    # through experiment, the target standbar displacement should be larger than +0.09
    target_standbar_displacement=detax+extra_compression_value 
    rospy.loginfo("target_standbar_displacement is:%s",str(target_standbar_displacement))
    return target_standbar_displacement

def rod_mechanism_holding(target_standbar_displacement):
    open_hold_to_ceil_flag= rospy.get_param("/renov_up_level/open_hold_to_ceil_flag")
    rospy.loginfo("%s is %s", rospy.resolve_name('open_hold_to_ceil_flag'), open_hold_to_ceil_flag)
    if open_hold_to_ceil_flag==1:
        flexbar_upwardsmotion_process()
        standbar_motion_process(target_standbar_displacement)

        read_line_encode_bottom = rospy.get_param("/renov_up_level/read_line_encode_bottom")
        read_line_l0_encoder_bottom = rospy.get_param("/renov_up_level/read_line_l0_encode_bottom")
        current_displacement=-(read_line_encode_bottom-read_line_l0_encoder_bottom)
        standbar_tracking_error=target_standbar_displacement-current_displacement
        rospy.logerr("standbar----target Distance is: %s",str(target_standbar_displacement))
        rospy.logerr("standbar----current Distance is:%s",str(current_displacement))
        rospy.logerr("standbar----Distance_error is:%s",str(standbar_tracking_error))
        pid_tolerance_error_standbar=rospy.get_param("/renov_up_level/pid_tolerance_error_standbar")
        top_limit_switch_status=rospy.get_param("/renov_up_level/top_limit_switch_status")
        if top_limit_switch_status==1 or abs(standbar_tracking_error)<pid_tolerance_error_standbar:
            standbar_motion_end()
            flexbar_upwardsmotion_end()
            os.system('rosparam set /renov_up_level/open_hold_to_ceil_flag 0')



def rodclimb_mechanism_motion(target_rotation_angle,target_distance):
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

def rotation_motion(target_rotation_angle):
    rotation_enable(target_rotation_angle)
    rospy.loginfo("the motion of rotation mechanism in process")

    # output tracking errors
    rotation_joint_line_equation_k=rospy.get_param("/renov_up_level/rotation_joint_line_equation_k")
    rotation_joint_line_equation_b=rospy.get_param("/renov_up_level/rotation_joint_line_equation_b")
    pid_tolerance_error_rotation=rospy.get_param("/renov_up_level/pid_tolerance_error_rotation")
    current_rotation_angle=rospy.get_param("/renov_up_level/rotation_abs_encode")*rotation_joint_line_equation_k+rotation_joint_line_equation_b
    rotation_trackingerror= target_rotation_angle-current_rotation_angle
    rospy.logerr(("%s is %f"%("rotation target angle",target_rotation_angle)))
    rospy.logerr(("%s is %f"%("rotation current angle",current_rotation_angle)))
    rospy.logerr(("%s is %f"%("rotation tracking error",rotation_trackingerror)))
    if abs(rotation_trackingerror)<=pid_tolerance_error_rotation:
        rotation_disable()
        rospy.loginfo("the motion of rotation mechanism is closed")

def climb_motion(target_distance):
    climb_enable(target_distance)
    rospy.loginfo("the homing of climbing mechanism in process")

    # output tracking errors
    pid_tolerance_error_climb=rospy.get_param("/renov_up_level/pid_tolerance_error_climb")
    read_line_l0_encode=rospy.get_param("/renov_up_level/read_line_l0_encode")
    climb_tracking_error=read_line_l0_encode+target_distance-rospy.get_param("/renov_up_level/read_line_encode")
    rospy.logerr(("%s is %f"%("tracking error",climb_tracking_error)))
    if abs(climb_tracking_error)<=pid_tolerance_error_climb:
        climb_disable()
        rospy.loginfo("the motion of climbing mechanism is closed")



class FLEX3DOFROBOTHOME():
    def __init__(self,nodename):
        self.nodename=nodename
    def Init_node(self):
        rospy.init_node(self.nodename)
    
def main():
    nodename="jackup_mechanism_homing"
    hc3dof=FLEX3DOFROBOTHOME(nodename)
    hc3dof.Init_node()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        home_climb_flex_bar = rospy.get_param("/renov_up_level/home_climb_flex_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
        if home_climb_flex_bar==1:
            jackup_mechanism_homing()
            # jackup_mechanism_homing(target_standbar_displacement,target_rotation_angle,target_climb_distance)
            # rodmechanism_homing(target_standbar_displacement)
        rate.sleep()

if __name__=="__main__":
    main()



