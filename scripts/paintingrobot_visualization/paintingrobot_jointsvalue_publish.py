#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from transfer import *
from aubo_kinematics import *
from Quaternion import *
from painting_robot_demo.msg import catersian_vel,physical_para
from sensor_msgs.msg import JointState
from tf_conversions import transformations
from math import pi

class Renovationrobot_joints_pub():
    def __init__(self):
        self.aubo_joints_sub=rospy.Subscriber('aubo_joints', JointState, self.obtain_aubo_joints, queue_size=10)
        self.mobileplatform_joints_sub=rospy.Subscriber(self.obtain_mobileplatform_states, queue_size=10)
        self.paintingrobot_joints_pub=rospy.Publisher('paintingrobot_joints', JointState, queue_size=10)

        self.mobile_platform_joints_value=[0.0,0.0,0.0]
        self.jackup_mechanism_joints_value=[0.0,0.0]
        self.aubo_arm_joints_value=[0.0,0.0,0.0,0.0,0.0,0.0]

        self.listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

    def obtain_auboarm_states(self,msg):
        self.aubo_joints_value[0]=msg.position[0]
        self.aubo_joints_value[1]=msg.position[1]
        self.aubo_joints_value[2]=msg.position[2]
        self.aubo_joints_value[3]=msg.position[3]
        self.aubo_joints_value[4]=msg.position[4]
        self.aubo_joints_value[5]=msg.position[5]

    def obtain_jackupmechanism_states(self):
        rospy.get_param('/renov_up_level/read_line_encode')
        rospy.get_param('/renov_up_level/read_line_encode')
        rospy.get_param('/renov_up_level/read_line_encode')
        

    def obtain_mobileplatform_states(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return None
        euler = transformations.euler_from_quaternion(rot)
        self.mobile_platform_joints_value[0] = trans[0]
        self.mobile_platform_joints_value[1] = trans[1]
        self.mobile_platform_joints_value[2] = euler[2] / pi * 180

    def obtain_paintingrobot_states(self):
        paintingrobot_joints=JointState()
        paintingrobot_joints.header.stamp=rospy.Time.now()

        paintingrobot_joints.name[0]='base_joint1'
        paintingrobot_joints.position[0]=self.mobile_platform_joints_value[0]
        paintingrobot_joints.name[1]='base_joint2'
        paintingrobot_joints.position[1]=self.mobile_platform_joints_value[1]
        paintingrobot_joints.name[2]='mobilebase_joint'
        paintingrobot_joints.position[2]=self.mobile_platform_joints_value[2]

        paintingrobot_joints.name[3]='rodclimbing_joint1'
        paintingrobot_joints.position[3]=self.jackup_mechanism_joints_value[0]
        paintingrobot_joints.name[4]='rodclimbing_joint2'
        paintingrobot_joints.position[4]=self.jackup_mechanism_joints_value[1]

        paintingrobot_joints.name[5]='shoulder_joint'
        paintingrobot_joints.position[5]=self.aubo_arm_joints_value[0]
        paintingrobot_joints.name[6]='upperArm_joint'
        paintingrobot_joints.position[6]=self.aubo_arm_joints_value[1]
        paintingrobot_joints.name[7]='foreArm_joint'
        paintingrobot_joints.position[7]=self.aubo_arm_joints_value[2]
        paintingrobot_joints.name[8]='wrist1_joint'
        paintingrobot_joints.position[8]=self.aubo_arm_joints_value[3]
        paintingrobot_joints.name[9]='wrist2_joint'
        paintingrobot_joints.position[9]=self.aubo_arm_joints_value[4]
        paintingrobot_joints.name[10]='wrist3_joint'
        paintingrobot_joints.position[10]=self.aubo_arm_joints_value[5]

        self.paintingrobot_joints_pub.publish(paintingrobot_joints)
        


if __name__ == '__main__':
    rospy.init_node('endeffectorpose_computation', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rate.sleep()





# class robot_pose_computation():
#     def __init__(self):
#         self.T1=np.eye(4)
#         self.T2=np.eye(4)
#         self.T3=np.eye(4)
#         self.wholerobot_pose=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
#         self.aubo_pose_sub = rospy.Subscriber('aubo_pose',Pose, manipulator_computation)

#     # part one: mobile platform frame to map frame
#     def tf_computation(self):
#         try:
#             (mobile_trans, mobile_quaternion) = listener.lookupTransform('/base_link', '/aubo_baselink', rospy.Time(0))
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue
#         # obtain trans
#         mobile_trans
#         # quternion to euler

#         # final: obtain T1 and z-yaw angle

#     # part two: aubo manipulator base to mobile platform frame: height/angle is obtained from the linear/rotation encoder respectively
#     def rodclimbing_computation(self):
#         constantvalue=0.5
#         rotation_length=0.6
#         linear_encoder_length=rospy.get_param("")
#         rotation_encoder_angle=rospy.get_param("")
#         tran_x=rotation_length*math.cos(rotation_encoder_angle)
#         tran_y=rotation_length*math.sin(rotation_encoder_angle)
#         tran_z=linear_encoder_length+constantvalue

#         rot_mat=rotz(rotation_encoder_angle)
#         # obtain T2 and z-yaw angle

#     # part three: aubo manipulator end effector to base frame
#     def manipulator_computation(self,data):
#         p = np.array([data.position.x, data.position.y, data.position.z])
#         q = np.array([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
#         # obtain T3 and rpy angles
#         mat_generation = pose2mat()
#         self.T3 = mat_generation.mat4x4(p, q)

#     def wholerobot_computation(self):
#         wholerobot_T=np.array(self.T1*self.T2*self.T3)
#         tran_x=wholerobot_T[0][3]
#         tran_y=wholerobot_T[1][3]
#         tran_z=wholerobot_T[2][3]
#         # obtain rpy angles and T matrix
#     listener = tf.TransformListener()








