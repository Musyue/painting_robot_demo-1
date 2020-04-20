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
    while not rospy.is_shutdown():
        home_climb_flex_bar = rospy.get_param("home_climb_flex_bar")
        rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
        write_flex_pole_motor_down = rospy.get_param("write_flex_pole_motor_down")
        rospy.loginfo("%s is %s", rospy.resolve_name('write_flex_pole_motor_down'), write_flex_pole_motor_down)
        if home_climb_flex_bar==1:

            os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 0')
            os.system('rosparam set /renov_up_level/climb_distance_tracking_over 0')

            # the downwards motion of flex pole 
            os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 2')
            time.sleep(5)
            rospy.loginfo("waiting for flex bar go down to start point")
            os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 0')

            # reset motion of climb mechanism
            os.system('rosparam set /renov_up_level/enable_climb_control 1')
            os.system('rosparam set /renov_up_level/enable_climb_control 0')
            os.system('rosparam set /renov_up_level/open_climb_flag 0')
            rospy.loginfo("reset motion of climbing mechanism begin")
            os.system('rosparam set /renov_up_level/distance_climb_control 0')
            time.sleep(5)
            rospy.loginfo("reset motion of climbing mechanism in process")

            # reset motion of rotation mechanism 
            os.system('rosparam set /renov_up_level/enable_control_rotation 1')
            os.system('rosparam set /renov_up_level/enable_control_rotation 0')
            os.system('rosparam set /renov_up_level/open_rotation_flag 1')
            rospy.loginfo("reset motion of rotation mechanism begin")
            os.system('rosparam set /renov_up_level/rad_control_rotation 0')
            time.sleep(5)
            rospy.loginfo("reset motion of rotation mechanism in process")


            # reset motion of standing bar 
            os.system('rosparam set /renov_up_level/open_hold_flag 1')
            os.system('rosparam set /renov_up_level/enable_control_stand_bar 1')
            os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
            os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 1')
            rospy.loginfo("reset motion of standing bar mechanism begin")
            os.system('rosparam set /renov_up_level/distance_control_stand_bar 0')
            time.sleep(5)
            rospy.loginfo("reset motion of standing bar in process")

            
            os.system('rosparam set /renov_up_level/open_climb_flag 0')
            os.system('rosparam set /renov_up_level/enable_climb_control 2')
            os.system('rosparam set /renov_up_level/enable_climb_control 0')
            time.sleep(0.05)

            os.system('rosparam set /renov_up_level/open_rotation_flag 0')
            os.system('rosparam set /renov_up_level/enable_control_rotation 2')
            os.system('rosparam set /renov_up_level/enable_control_rotation 0')
            time.sleep(0.05)

            os.system('rosparam set /renov_up_level/open_hold_flag 0')
            os.system('rosparam set /renov_up_level/enable_control_stand_bar 2')
            os.system('rosparam set /renov_up_level/enable_control_stand_bar 0')
            os.system('rosparam set /renov_up_level/enable_second_control_stand_bar 0')
            time.sleep(0.05)
           

            rospy.set_param('home_climb_flex_bar',0)
            os.system('rosparam set /renov_up_level/home_climb_flex_bar 0')
            rospy.set_param('mobile_tracking_stop_flag',0)
            os.system('rosparam set /renov_up_level/mobile_tracking_stop_flag 0')
            rospy.logerr("homing program over--  go to next mobile way point-------")
            os.system('rosparam set /renov_up_level/hold_distance_tracking_over 0')
            os.system('rosparam set /renov_up_level/climb_distance_tracking_over 0')
            os.system('rosparam set /renov_up_level/rotation_distance_tracking_over 0')
        rate.sleep()

if __name__=="__main__":
    main()
