#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

class MoveItObstaclesDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('paintingrobot_planningscene')
        # 初始化场景对象
        scene = PlanningSceneInterface()
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
        # 等待场景准备就绪
        rospy.sleep(1)                
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'map'
        stl_id='stl'
        scene.remove_world_object(stl_id)   
        stl_size=[1.0,1.0,1.0]
        stl_pose = PoseStamped()
        stl_pose.header.frame_id = reference_frame
        stl_pose.pose.position.x = 0.0
        stl_pose.pose.position.y = 0.0
        stl_pose.pose.position.z = -2.4701
        stl_pose.pose.orientation.w = 1.0
        scene.add_mesh(stl_id,stl_pose,'/data/ros/renov_robot_ws/src/painting_robot_demo/matlab/bim_document/001_1.stl')   
        self.setColor(stl_id, 0.8, 0.4, 0, 1.0)
        # 将场景中的颜色设置发布
        self.sendColors()    
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        
    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()
        # 需要设置规划场景是否有差异     
        p.is_diff = True
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItObstaclesDemo()
    except KeyboardInterrupt:
        raise
    

    
