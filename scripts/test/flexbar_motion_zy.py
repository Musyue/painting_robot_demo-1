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
    
def flexbar_upwardsmotion():
    # upwards motion of flexbar mechanism 
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 1')
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_up 0')
    rospy.loginfo("upwards motion of flexbar mecahnism")
def flexbar_downwardsmotion():
    # downwards motion of flexbar mechanism
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 2')
    os.system('rosparam set /renov_up_level/write_flex_pole_motor_down 0')
    rospy.loginfo("downwards motion of flexbar mechanism")

def main():
    nodename="climb_flex_bar_home_node"

    hc3dof=FLEX3DOFROBOTHOME(nodename)

    hc3dof.Init_node()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # home_climb_flex_bar = rospy.get_param("/renov_up_level/home_climb_flex_bar")
        # rospy.loginfo("%s is %s", rospy.resolve_name('home_climb_flex_bar'), home_climb_flex_bar)
        # if home_climb_flex_bar==1:
        # flexbar_upwardsmotion()
        flexbar_downwardsmotion()
        rate.sleep()
if __name__=="__main__":
    main()
