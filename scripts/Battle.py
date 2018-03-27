# -*- coding: utf-8 -*-
import sys
import numpy as np
from PIL import Image
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from teleop_controller.msg import ShootCmd, EnemyPos
import time
import math



class BattleEnv():
    def __init__(self):
        self.action_space = ['N', 'E', 'W', 'S', 'NE', 'NW', 'SE', 'SW', 'shoot']
        self.map = np.array(Image.open("icra.pgm"))
        self.shoot_pub = rospy.Publisher('shoot_cmd', ShootCmd, queue_size=1)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        self.navgoal = PoseStamped()
        self.navgoal.header.frame_id = 'map'
        #self.navgoal.pose.orientation.w = 1.0
        self.shoot = ShootCmd()
        self.MyPose = {'x': 0, 'y': 0, 'theta': 0}
        self.EnemyPose = EnemyPos()
        self.enemyNew = False
        self.totalhurt = [0, 0]
        rospy.init_node('BattleSim')

    def reset(self):
        # 重置仿真环境
        initialstates = np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0])  # [xs,ys,dirs,enemy_yaw,enemy_pitch,enemy_dist,isTarget,isTargeted]
        self.totalhurt = [0, 0]
        return initialstates

    # def gettransform(self):

    def getSelfPoseCallback(self, data):
        # data = Odometry()
        self.MyPose['x'] = data.pose.pose.position.x + 1.0
        self.MyPose['y'] = data.pose.pose.position.y + 1.0
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        angle = math.atan2(2 * (qx * qy + qz * qw), qw * qw + qx * qx - qy * qy - qz * qz) * 180 / 3.1415926
        self.MyPose['theta'] = angle
        # print('Get Odom')
        # print(qx, qy, qz, qw, angle)

    def getEnemyPoseCallback(self, data):
        # data = EnemyPos()
        self.EnemyPose.enemy_yaw = data.enemy_yaw
        self.EnemyPose.enemy_pitch = data.enemy_pitch
        self.EnemyPose.enemy_dist = data.enemy_dist
        self.enemyNew = True
        # print('Get enemy pose')

    '''
    仿真步进，根据状态动作输出回报
    '''

    '''def step(self, action, observation):
        # 判断目标的点是否可达
        if self.isActionAvaliable(action, observation):
            # take move or shoot
            self.shoot_pub.publish(self.shoot)
            self.goal_pub.publish(self.navgoal)

            rospy.Subscriber('/odom', Odometry, self.getSelfPoseCallback)
            time.sleep(0.1)
            # print('flag0')
            rospy.Subscriber('/enemy_pos', EnemyPos, self.getEnemyPoseCallback)
            time.sleep(0.1)
            # print('flag1')

            observationnew = self.calcAttackArea(observation)  # 计算攻击区域

            isshoot, ishurt = self.calcShootProb(observationnew, action)  # 计算命中率

            if observationnew[6] == 1:  # target enemy
                movereward = 1
                if observationnew[7] == 0:  # not targeted
                    movereward = 1.5
            elif observationnew[7] == 0:  # both side not target
                movereward = -1
            elif observationnew[7] == 1:  # only be targeted
                movereward = -1.5
            else:
                movereward = 0

            if isshoot == 1:  # shoot enemy
                shootreward = 2
            else:
                shootreward = -1

            if ishurt == 1:  # be shooted by enemy
                hurtreward = -2
            else:
                hurtreward = 0
        else:
            # take shoot or nothing
            self.shoot_pub.publish(self.shoot)
            observationnew = self.calcAttackArea(observation)  # 计算攻击区域
            isshoot, ishurt = self.calcShootProb(observationnew, action)
            movereward = -1

            if isshoot == 1:  # shoot enemy
                shootreward = 2
            else:
                shootreward = -1

            if ishurt == 1:  # be shooted by enemy
                hurtreward = -2
            else:
                hurtreward = 0

        reward = movereward + shootreward + hurtreward
        done = False
        if self.totalhurt[0] > 1500 or self.totalhurt[1] > 1500:
            done = True
        return observationnew, reward, done'''

    def isActionAvaliable(self, goal):
        #orin = Quaternion()
        #goal = PoseStamped()
        ok = False
        x_goal = int(goal.pose.position.x / 0.05)
        y_goal = int(goal.pose.position.y / 0.05)
        if self.map[101 - y_goal, x_goal] == 255:  # 是否在地图可行区域
            self.navgoal.pose.position.x = x_goal * 0.05  # 101X161地图栅格
            self.navgoal.pose.position.y = y_goal * 0.05
            self.navgoal.pose.orientation.x = goal.pose.orientation.x
            self.navgoal.pose.orientation.y = goal.pose.orientation.y
            self.navgoal.pose.orientation.z = goal.pose.orientation.z
            self.navgoal.pose.orientation.w = goal.pose.orientation.w
            ok = True
        else:
            pass
        '''if shoot == 1:  # Shoot
            self.shoot.shoot_cmd = 1
        else:
            pass'''
        return ok
        # return states


    '''
    计算攻击区域范围，通过位置状态判断敌人是否进入攻击区域，返回攻击所造成伤害
    '''

    def calcAttackArea(self, observation):
        observation[0] = self.MyPose['x']
        observation[1] = self.MyPose['y']
        observation[2] = self.MyPose['theta']
        observation[3] = self.MyPose['x'] + np.cos((self.MyPose['theta'] + self.EnemyPose.enemy_yaw) * 180 / 3.1416)
        observation[4] = self.MyPose['y'] + np.sin((self.MyPose['theta'] + self.EnemyPose.enemy_yaw) * 180 / 3.1416)
        observation[5] = self.EnemyPose.enemy_dist
        if self.enemyNew and (
                self.EnemyPose.enemy_yaw < 90 or self.EnemyPose.enemy_yaw > -90) and self.EnemyPose.enemy_dist < 3:
            observation[6] = 1  # targeted
            observation[7] = 1
        else:
            observation[6] = 0
            observation[7] = 0
        return observation

    '''
    计算射击命中率
    '''

    def calcShootProb(self, observation, action):
        p = 0
        if observation[6] == 0 or action[4] == 0:  # 未瞄或未射击
            p = 0  # binomial distribution
        elif observation[5] > 3:
            p = 0.4
        elif observation[5] > 2 and observation[5] < 3:
            p = 0.6
        elif observation[5] > 1 and observation[5] < 2:
            p = 0.8
        elif observation[5] > 0 and observation[5] < 1:
            p = 0.9
        if observation[6] == 1 and action[4] == 0:
            pp = 0.25
        else:
            pp = p
        isshoot = np.random.binomial(1, p)
        ishurt = np.random.binomial(1, pp)
        shootreward = isshoot * 50
        hurt = ishurt * 50
        self.totalhurt[0] = self.totalhurt[0] + shootreward
        self.totalhurt[1] = self.totalhurt[1] + hurt
        return isshoot, ishurt

    '''计算收到伤害'''

    def getDamage(self):
        pass

    def render(self):
        pass


if __name__ == '__main__':
    env = BattleEnv()
    initialobservation = env.reset()
    action = [1, 0, 0, 0, 0]
    #env.step(action, initialobservation)
    # Image._show(env.map)
    print(env.map)
