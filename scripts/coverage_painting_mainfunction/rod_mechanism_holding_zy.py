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
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        mobile_platform_tracking_over_flag= rospy.get_param("/renov_up_level/mobile_platform_tracking_over_flag")
        rospy.loginfo("%s is %s", rospy.resolve_name('mobile_platform_tracking_over_flag'), mobile_platform_tracking_over_flag)
        if mobile_platform_tracking_over_flag==1:
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
                os.system('rosparam set /renov_up_level/mobile_platform_tracking_over_flag 0')
                os.system('rosparam set /renov_up_level/rodmechanism_holding_over_flag 1')
                break
        rate.sleep()


class FLEX3DOFROBOTHOLD():
    def __init__(self,nodename):
        self.nodename=nodename
    def Init_node(self):
        rospy.init_node(self.nodename)

def main():
    nodename="climb_flex_bar_hold_node"
    hc3dof=FLEX3DOFROBOTHOLD(nodename)
    hc3dof.Init_node()

    target_standbar_displacement=holding_rod_mechanism_target_standbar_displacement_computation()
    rod_mechanism_holding(target_standbar_displacement)

if __name__=="__main__":
    main()
