# !/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Battle import BattleEnv
from teleop_control import Controller
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW
import time



GIMBAL_RELAX = 0
GIMBAL_INIT = 1
GIMBAL_NO_ARTI_INPUT = 2
GIMBAL_FOLLOW_ZGYRO = 3
GIMBAL_TRACK_ARMOR = 4
GIMBAL_PATROL_MODE = 5
GIMBAL_SHOOT_BUFF = 6
GIMBAL_POSITION_MODE = 7



CHASSIS_RELAX = 0
CHASSIS_STOP = 1
MANUAL_SEPARATE_GIMBAL = 2
MANUAL_FOLLOW_GIMBAL = 3
DODGE_MODE = 4
AUTO_SEPARATE_GIMBAL = 5
AUTO_FOLLOW_GIMBAL = 0

#启动控制器和环境类
controller = Controller()
env = BattleEnv()
#定义控制命令
vel = Twist()
nav_goal = PoseStamped()
shoot_cmd = ShootCmd()
mode = ModeSW()
#接受机器人状态消息
rospy.Subscriber('/odom', Odometry, env.getSelfPoseCallback)
time.sleep(0.1)
# print('flag0')
rospy.Subscriber('/enemy_pos', EnemyPos, env.getEnemyPoseCallback)
time.sleep(0.1)

print 'self location is x=%s\ty=%s\t yaw=%s\n' % (env.MyPose['x'], env.MyPose['y'], env.MyPose['theta'])
print 'enemy pose is yaw=%s\tpitch=%s\tdistance=%s\n' % (env.EnemyPose.enemy_yaw, env.EnemyPose.enemy_pitch, env.EnemyPose.enemy_dist)
#发布目标点
nav_goal.header.frame_id = 'map'
nav_goal.pose.position.x = 1
nav_goal.pose.position.y = 1

if env.isActionAvaliable(nav_goal):#判断目标点是否可行
    controller.send_goal(env.navgoal)#发送目标
    print 'sending goal'
else:
    pass
#切换模式
mode.chassis_mode = AUTO_FOLLOW_GIMBAL
mode.gimbal_mode = GIMBAL_POSITION_MODE
controller.mode_switch(mode)#发送模式
#发弹命令
shoot_cmd.fric_wheel_spd = 1500#摩擦轮转速设置
shoot_cmd.fric_wheel_run = 1#开关摩擦轮
shoot_cmd.shoot_cmd = 1#单发命令
shoot_cmd.c_shoot_cmd =0#连发命令
controller.shoot(shoot_cmd)#发布射击命令
