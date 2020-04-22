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


def climb_motion(distance):
    home_climb_flex_bar = rospy.get_param("/renov_up_level/home_climb_flex_bar")
    rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
    pid_tolerance_error_climb=rospy.get_param("/renov_up_level/pid_tolerance_error_climb")
    if home_climb_flex_bar==1:
        # the motion command of climb mechanism
        os.system('rosparam set /renov_up_level/enable_climb_control 1')
        os.system('rosparam set /renov_up_level/enable_climb_control 0')
        os.system('rosparam set /renov_up_level/open_climb_flag 1')
        os.system('rosparam set /renov_up_level/distance_climb_control '+str(distance))
        rospy.loginfo("the homing of climbing mechanism in process")

        # close the moiton of climb mechanism
        climb_tracking_error=0.62+distance-rospy.get_param("/renov_up_level/read_line_encode")
        rospy.loginfo(("%s is %f"%("tracking error",climb_tracking_error)))
        if abs(climb_tracking_error)<=pid_tolerance_error_climb:
            os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
            os.system('rosparam set /renov_up_level/climb_distance_tracking_over 0')
            os.system('rosparam set /renov_up_level/open_climb_flag 0')
            os.system('rosparam set /renov_up_level/enable_climb_control 2')
            os.system('rosparam set /renov_up_level/enable_climb_control 0')
            rospy.loginfo("the motion of climbing mechanism is closed")

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
    distance=0.1
    while not rospy.is_shutdown():
        climb_motion(distance)
        rate.sleep()

if __name__=="__main__":
    main()


# climb_distance_tracking_over=rospy.get_param("/renov_up_level/climb_distance_tracking_over")
# rospy.loginfo("%s is %s", rospy.resolve_name('climb_distance_tracking_over'), climb_distance_tracking_over)
# distance_climb_control=rospy.get_param("/renov_up_level/distance_climb_control")
# if climb_distance_tracking_over==1:
# rospy.loginfo("%s is %s", rospy.resolve_name('distance_climb_control'), distance_climb_control)
# if distance_climb_control==distance:
#     os.system('rosparam set /renov_up_level/open_climb_flag 1')
#     rospy.loginfo("the homing of climbing mechanism in process")
# time.sleep(1)
# climb_distance_tracking_over=rospy.get_param("/renov_up_level/climb_distance_tracking_over")
# rospy.loginfo("%s is %s", rospy.resolve_name('climb_distance_tracking_over'), climb_distance_tracking_over)