#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys, os
from math import *
import numpy as np
import numpy.matlib
import moveit_commander
import scipy.io as io
import tf

coverage_planner_path="/home/zy/catkin_ws/src/paintingrobot/painting_robot_demo/scripts"
mat_path="/home/zy/catkin_ws/src/paintingrobot/painting_robot_demo/data/data3.mat" 

sys.path.append(coverage_planner_path)
from paintingrobot_planning.coverage_planning_offline1 import *
from paintingrobot_planning.robotic_functions.transfer import *
from paintingrobot_planning.robotic_functions.aubo_kinematics import *
from paintingrobot_planning.robotic_functions.Quaternion import *

from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA
from waypoints_paths_visualization_functions import *

from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose


# class Renovationrobot_planningresult_collision_check():


class Renovationrobot_motion():
    def __init__(self):
        self.interval=0.10 #rospy.get_param('mat_interval')
        self.distance=0.10
        self.parameterx=0.430725381079
        self.parametery=-0.00033063639818
        self.parameterz=1.32061628232

    def renovationrobot_joints_computation_0(self,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell):
        # computation of target joints of mobile platform
        theta_z=manipulatorbase_targetpose_onecell[0][5]
        deltax=self.parameterx*cos(theta_z)-self.parametery*sin(theta_z)
        deltay=self.parameterx*sin(theta_z)+self.parametery*cos(theta_z)
        mobileplatform_targetjoints=[manipulatorbase_targetpose_onecell[0][0]-deltax,(manipulatorbase_targetpose_onecell[0][1]-deltay),theta_z]

        # computation of target joints of rodclimbing_robot
        rodclimbing_robot_targetjoints=[manipulatorbase_targetpose_onecell[0][2]-self.parameterz,0.0]

        # computation of inverse joints of manipulator
        aubo_joints_list=np.array([-0.2852509833270265, -0.5320376301933496, 1.3666906155038931, -1.2428644078925508, -1.856047310121923,1.5707963267948966])
        previous_aubo_joints=aubo_joints_list

        for i in range(len(manipulatorendeffector_targetpose_onecell)-1):
            p1=np.array([manipulatorendeffector_targetpose_onecell[i][0],manipulatorendeffector_targetpose_onecell[i][1],manipulatorendeffector_targetpose_onecell[i][2]])
            p2=np.array([manipulatorendeffector_targetpose_onecell[i+1][0],manipulatorendeffector_targetpose_onecell[i+1][1],manipulatorendeffector_targetpose_onecell[i+1][2]])
            distance=sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)
            p=np.zeros(3)
            num = int(distance / self.interval)+1
            # if i==len(manipulatorendeffector_targetpose_onecell)-2:
            #     num = int(distance / self.interval)+2
            # else:
            #     num = int(distance / self.interval)+1
            for j in range(num):
                p[0] = p1[0] + (p2[0] - p1[0]) / distance * self.interval * j
                p[1] = p1[1] + (p2[1] - p1[1]) / distance * self.interval * j
                p[2] = p1[2] + (p2[2] - p1[2]) / distance * self.interval * j
                q = np.array([manipulatorendeffector_targetpose_onecell[j][3], manipulatorendeffector_targetpose_onecell[j][4],manipulatorendeffector_targetpose_onecell[j][5]])
                # print("p is:",p)
                # print("q is:",q)
                T_mat_generation = pose2mat()
                mat = T_mat_generation.mat4x4(p, q)
                mat1 = np.ravel(mat)
                mat2 = mat1.tolist()
                aubo_arm = Aubo_kinematics()
                aubo_joints_onepoint = aubo_arm.GetInverseResult(mat2, previous_aubo_joints)
                previous_aubo_joints = aubo_joints_onepoint
                aubo_joints_list = np.append(aubo_joints_list, aubo_joints_onepoint, axis=0)

        points_num=len(aubo_joints_list)/6
        for i in range(points_num):
            aubo_joints=np.array(aubo_joints_list[6*i:6*i+6])
        aubo_targetjoints = aubo_joints_list.reshape(len(aubo_joints_list) / 6, 6)
        return mobileplatform_targetjoints, rodclimbing_robot_targetjoints, aubo_targetjoints

    def mobile_platform_rviz_motion(self, mobileplatform_targetjoints):
        "def of mobile platform motion groups" 
        mobileplatform = moveit_commander.MoveGroupCommander('mobileplatform')
        # mobileplatform.set_goal_joint_tolerance(0.001)
        # mobileplatform.set_max_acceleration_scaling_factor(1)
        # mobileplatform.set_max_velocity_scaling_factor(1)

        "motion of mobile platform" 
        print("mobileplatform_targetjoints=",mobileplatform_targetjoints)
        mobileplatform.set_joint_value_target(mobileplatform_targetjoints)
        mobileplatform_state=mobileplatform.go()
        rospy.sleep(0.2)
        if mobileplatform_state==False:
            rospy.logerr("mobile platform planning is failed !")
        return mobileplatform_state

    def rodclimbing_mechanism_rviz_motion(self, mobileplatform_state, rodclimbing_robot_targetjoints):
        "definition of rod climbing mechanism moveit groups"
        if mobileplatform_state==True:
            rodclimbing_robot = moveit_commander.MoveGroupCommander('rodclimbing_robot')
            # rodclimbing_robot.set_goal_joint_tolerance(0.001)
            # rodclimbing_robot.set_max_acceleration_scaling_factor(1)
            # rodclimbing_robot.set_max_velocity_scaling_factor(1)

            "motion of rod climbing mechanism"
            print("rodclimbing_robot_targetjoints=",rodclimbing_robot_targetjoints)
            rodclimbing_robot.set_joint_value_target(rodclimbing_robot_targetjoints)
            rodclimbing_robot_state=rodclimbing_robot.go()
            rospy.sleep(0.2)
            if rodclimbing_robot_state==False:
                rospy.logerr("rod climbing mechanism planning is failed !")
            return rodclimbing_robot_state
        else: 
            str1="rod climbing mechanism won't be planned"
            rospy.logerr(str1)
            return str1

    def aubo_rviz_motion(self ,mobileplatform_state, rodclimbing_robot_state,aubo_joints_list):
        if mobileplatform_state==True and rodclimbing_robot_state==True:
            "motion of aubo5 robotic arm"
            arm = moveit_commander.MoveGroupCommander('aubo5')
            # arm.set_goal_joint_tolerance(0.01)
            # arm.set_max_acceleration_scaling_factor(1)
            # arm.set_max_velocity_scaling_factor(1)

            points_num=len(aubo_joints_list)
            for i in range(points_num):
                # motion of manipulator
                aubo_joints=[aubo_joints_list[i][0],aubo_joints_list[i][1],aubo_joints_list[i][2],aubo_joints_list[i][3],aubo_joints_list[i][4],aubo_joints_list[i][5]] #np.array()
                # print("aubo_joints is %s"%str(aubo_joints))
                arm.set_joint_value_target(aubo_joints)
                arm_state=arm.go()
                if arm_state==False:
                    rospy.logerr("manipulator planning is failed !")
                    break
            return arm_state
        else: 
            str1="manipulator won't be planning!"
            rospy.logerr(str1)
            return str1
    
    def homing_motion(self):
        "definition of three moveit groups" 
        mobileplatform = moveit_commander.MoveGroupCommander('mobileplatform')
        rodclimbing_robot= moveit_commander.MoveGroupCommander('rodclimbing_robot')
        arm = moveit_commander.MoveGroupCommander('aubo5')
        "these groups moves to initial poses" 
        mobileplatform.set_named_target('home1')
        mobileplatform.go()
        # rospy.sleep(1)
        rodclimbing_robot.set_named_target('home2')
        rodclimbing_robot.go()
        # rospy.sleep(1)
        arm.set_named_target('home3')
        arm.go()
        # rospy.sleep(1)

    def renovation_robot_simulation_nonvisualization(self,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell):
        "computation of target joints of mobile platform and rodclimbing mechanisam and inverse joints of manipulator" 
        mobileplatform_targetjoints, rodclimbing_robot_targetjoints, aubo_targetjoints=self.renovationrobot_joints_computation_0(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)
        
        "motion of mobile platform"
        mobileplatform_state=self.mobile_platform_rviz_motion(mobileplatform_targetjoints)

        "motion of rod climbing mechanism"
        rospy.loginfo("rodclimbing_robot_targetjoints is %s"%str(rodclimbing_robot_targetjoints))
        rodclimbing_robot_state=self.rodclimbing_mechanism_rviz_motion(mobileplatform_state,rodclimbing_robot_targetjoints)

        # mobileplatform_state=True
        # rodclimbing_robot_state=True
        "motion of industrial manipulator"
        arm_state=self.aubo_rviz_motion(mobileplatform_state,rodclimbing_robot_state, aubo_targetjoints)
    
    def mobile_platform_visualization(self,visualization_num,mobileplatform_targetjoints):
        # visualization of target mobile platform positions and mobile platform path
        maobileplatform_T1 = rotz(mobileplatform_targetjoints[2])
        maobileplatform_T2 = r2t(maobileplatform_T1)
        q0 = quaternion(maobileplatform_T2)
        frame = 'map'
        mobileplatform_targepositions=np.array([mobileplatform_targetjoints[0],mobileplatform_targetjoints[1],0, q0.s, q0.v[0, 0], q0.v[0, 1], q0.v[0, 2]])
        scale1=np.array([0.2,0.2,0.2])
        color1=np.array([1.0,0.0,0.0])
        marker1,visualization_num=targetpositions_visualization(mobileplatform_targepositions, frame, visualization_num, scale1, color1)
        marker_pub.publish(marker1)
        # rospy.sleep(0.2)
        return visualization_num

    def manipulatorbase_positions_visualization(self,visualization_num,manipulatorbase_targetpose_onecell):

        # manipulatorbase_targepositions=[]
        # manipulatorbase_targepositions.append(manipulatorbase_targetpose_onecell[0][0])
        # manipulatorbase_targepositions.append(manipulatorbase_targetpose_onecell[0][1])
        # manipulatorbase_targepositions.append(manipulatorbase_targetpose_onecell[0][2])
        # angles=tf.transformations.quaternion_from_euler(manipulatorbase_targetpose_onecell[0][3],manipulatorbase_targetpose_onecell[0][4],manipulatorbase_targetpose_onecell[0][5])
        # manipulatorbase_targepositions.append(angles[0])
        # manipulatorbase_targepositions.append(angles[1])
        # manipulatorbase_targepositions.append(angles[2])
        # manipulatorbase_targepositions.append(angles[3])

        maobileplatform_T1 = rotz(manipulatorbase_targetpose_onecell[0][5])
        maobileplatform_T2 = r2t(maobileplatform_T1)
        q0 = quaternion(maobileplatform_T2)
        manipulatorbase_targepositions=np.array([manipulatorbase_targetpose_onecell[0][0],manipulatorbase_targetpose_onecell[0][1],manipulatorbase_targetpose_onecell[0][2], q0.s, q0.v[0, 0], q0.v[0, 1], q0.v[0, 2]])

        frame = 'map'
        scale2 = np.array([0.1, 0.1, 0.25])
        color2 = np.array([0.0, 1.0, 0.0])
        visualization_num=visualization_num+1
        marker1,visualization_num = targetpositions_visualization(manipulatorbase_targepositions, frame, visualization_num, scale2, color2)
        marker_pub.publish(marker1)
        # rospy.sleep(0.2)
        return visualization_num

    def target_region_visualization(self,visualization_num,painting_targetregion_onecell):
        # visualization of target painting region
        frame = 'map'
        for i in range(len(painting_targetregion_onecell)):
            visualization_num = visualization_num + 1
            waypoints = painting_targetregion_onecell[i]
            waypoints[0] = waypoints[0] + self.distance*cos(theta_z)
            waypoints[1] = waypoints[1] + self.distance*sin(theta_z)
            waypoints[3] = waypoints[3] + self.distance*cos(theta_z)
            waypoints[4] = waypoints[4] + self.distance*sin(theta_z)

            print("targetregion_boundaries=:",waypoints)
            marker1,visualization_num = self.path2_visualization(waypoints, frame, visualization_num)
            marker_pub.publish(marker1)
        return visualization_num

    def target_path_visualization(self,visualization_num,manipulatorendeffector_targetpose_onecell):
        "visualization of planned paths of manipulator"       
        frame='map'
        visualization_num=visualization_num+1
        marker1,visualization_num=path1_visualization(manipulatorendeffector_targetpose_onecell,frame,visualization_num)
        marker_pub.publish(marker1)
        return visualization_num

    def aubo_rviz_motion_visulization(self,mobileplatform_state,rodclimbing_robot_state,aubo_joints_list):
        if mobileplatform_state==True and rodclimbing_robot_state==True:
            "motion of aubo5 robotic arm"
            arm = moveit_commander.MoveGroupCommander('aubo5')
            # arm.set_goal_joint_tolerance(0.01)
            # arm.set_max_acceleration_scaling_factor(1)
            # arm.set_max_velocity_scaling_factor(1)

            points_num=len(aubo_joints_list)/6
            for i in range(points_num):
                "motion of manipulator" 
                # aubo_joints=np.array(aubo_joints_list[6*i:6*i+6])
                aubo_joints=[aubo_joints_list[i][0],aubo_joints_list[i][1],aubo_joints_list[i][2],aubo_joints_list[i][3],aubo_joints_list[i][4],aubo_joints_list[i][5]] #np.array()
                arm.set_joint_value_target(aubo_joints)
                arm_state=arm.go()
        
                "visualization of manipulator tarversed points" 
                if arm_state==True:
                    aubo_arm_T = aubo_arm.aubo_forward(aubo_joints)
                    aubo_arm_T1 = np.mat(aubo_arm_T)
                    aubo_arm_T2 = aubo_arm_T1.reshape([4,4])
                    q0 = quaternion(aubo_arm_T2)
        
                    traversed_point=np.array([aubo_arm_T2[0,3]+self.distance,aubo_arm_T2[1,3],aubo_arm_T2[2,3],q0.s,q0.v[0, 0],q0.v[0, 1],q0.v[0, 2]])
                    frame = 'aubo_baselink'
                    scale2 = np.array([0.1, 0.1, 0.002])
                    color2 = np.array([0.0, 0.0, 1.0])
                    visualization_num=visualization_num+1
                    marker1,visualization_num = targetpositions_visualization(traversed_point, frame, visualization_num, scale2, color2)
                    marker_pub.publish(marker1)
                    print("traversed_point=:",traversed_point)
            rospy.sleep(0.2)

    def renovation_robot_simulation_motion_visualization(self, visualization_num, paintingrobotendeffector_targetpose_onecell, manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell):
        "computation of target joints of mobile platform and rodclimbing mechanisam and inverse joints of manipulator" 
        mobileplatform_targetjoints, rodclimbing_robot_targetjoints, aubo_targetjoints=self.renovationrobot_joints_computation_0(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)
        

        "visualization of target mobile platform base positions"
        visualization_num=visualization_num+1
        visualization_num=self.mobile_platform_visualization(visualization_num, mobileplatform_targetjoints)
        print("mobileplatform_targetjoints is:",mobileplatform_targetjoints)

        # "motion of mobile platform"
        # mobileplatform_state=self.mobile_platform_rviz_motion(mobileplatform_targetjoints)

        "visualization of target rod climbing mechanism base positions"
        visualization_num=visualization_num+1
        visualization_num=self.manipulatorbase_positions_visualization(visualization_num, manipulatorbase_targetpose_onecell)

        # "motion of rod climbing mechanism"
        # rodclimbing_robot_state=self.rodclimbing_mechanism_rviz_motion(mobileplatform_state,rodclimbing_robot_targetjoints)

        "visualization of manipulator paths"
        visualization_num=visualization_num+1
        visualization_num=self.target_path_visualization(visualization_num, paintingrobotendeffector_targetpose_onecell)

        # "motion of industrial manipulator"
        # arm_state=self.aubo_rviz_motion(mobileplatform_state,rodclimbing_robot_state, aubo_targetjoints)

        return visualization_num


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('paintingrobot_planner', anonymous=True)
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    data = io.loadmat(mat_path)
    manipulatorbase_targetpose=data['renovation_cells_manipulatorbase_positions']
    manipulatorendeffector_targetpose=data['manipulator_endeffector_positions_onpath']
    paintingrobotendeffector_targetpose=data['renovation_cells_waypioints_onwaypath']
    
    Paintrobot = Renovationrobot_motion()
    Paintrobot.homing_motion()
    visualization_num=1
    try:
        while not rospy.is_shutdown():
        # for i in range(1): # range(len(manipulatorbase_targetpose[0])):
        #     for j in range(1): # range(len(manipulatorbase_targetpose[0][i][0])):
        #         for k in range(1): # range(len(manipulatorbase_targetpose[0][i][0][j][0])):
            for i in range(len(manipulatorbase_targetpose[0])):
                for j in range(len(manipulatorbase_targetpose[0][i][0])):
                    for k in range(len(manipulatorbase_targetpose[0][i][0][j][0])):
                        manipulatorbase_targetpose_onecell= manipulatorbase_targetpose[0][i][0][j][0][k]
                        manipulatorendeffector_targetpose_onecell = manipulatorendeffector_targetpose[0][i][0][j][0][k]
                        paintingrobotendeffector_targetpose_onecell = paintingrobotendeffector_targetpose[0][i][0][j][0][k]
                    
                        print("the plane num is:",i+1)
                        print("the mobile positions num is:",j+1)
                        print("the rod climbing positions num is:",k+1)
                        
                        # print("manipulatorbase_targetpose_onecell=:", manipulatorbase_targetpose_onecell)
                        # print("manipulatorendeffector_targetpose_onecell=:",manipulatorendeffector_targetpose_onecell)
                        # print("paintingrobotendeffector_targetpose_onecell=:",paintingrobotendeffector_targetpose_onecell)
                        
                        # Paintrobot.renovation_robot_simulation_nonvisualization(manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)
                        visualization_num=Paintrobot.renovation_robot_simulation_motion_visualization(visualization_num,paintingrobotendeffector_targetpose_onecell,manipulatorbase_targetpose_onecell,manipulatorendeffector_targetpose_onecell)  
                        visualization_num=visualization_num+1

    except rospy.ROSInterruptException:
        pass
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

# -1.3192469452282336, -1.9515323742482475, -0.3584438970917431
# -1.5121965458002296, -2.4665765001628213, -0.3584438970917431
# -1.7051461463722257, -2.9816206260773948, -0.3584438970917431
# -1.8980957469442217, -3.4966647519919682, -0.3584438970917431
# -2.0647340383473094, -3.9414755880091006, -0.3584438970917431
# 2.048898875211897, -1.074114146268727, -1.8953594448649913
# 1.5276143002953437, -0.8987220371130715, -1.8953594448649913
# 1.077413985594685, -0.7472470337513697, -1.8953594448649913
# data1=[0,0,-0.3584438970917431]
# ('quaternion angles is:', array([ 0.        ,  0.        , -0.17826404,  0.98398269]))
# data2=[0,0, -1.8953594448649913]
# ('quaternion angles is:', array([ 0.        ,  0.       , -0.81206365,  0.58356887]))