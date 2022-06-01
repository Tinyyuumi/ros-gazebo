#!/usr/bin/env python
# coding:utf-8

import numpy as np
import math
import threading

class DWA():
    def __init__(self,pose,goal,map):
        # 地图比例尺
        self.scale = 0.1

        # 机器人初期状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
        # self.pose = [9,0.5,0,0,0]
        self.pose = pose
        self.goal = goal

        self.obstacleR = 0.3 # 机器人半径
        self.dt = 0.1 # 一次仿真时间[s]

        # 机器人运动学模型参数
        # 最高速度[m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],
        # 速度分辨率[m/s],转速分辨率[rad/s]]
        self.Kinematic = [0.5, 2.0, 1.0, 2.0, 0.02,0.1]

        # 评价函数参数 [heading,dist,velocity,predictDT]
        # 航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
        self.evalParam = [0.08, 0.1 ,0.05, 3.0]
        
        # 地图
        self.map = map

        # 障碍物距离检测时窗口大小
        self.size = 8

    def evolve(self):
        while(1):
            # # 定位更新
            # self.pose[0] = self.pose[0] + self.dt*math.cos(self.pose[2])*self.pose[3] 
            # self.pose[1] = self.pose[1] + self.dt*math.sin(self.pose[2])*self.pose[3]
            # self.pose[2] = self.pose[2] + self.dt*self.pose[4]

            # # 判断是否达到目的地
            # dis = math.sqrt((position[0] - self.scale*keypath[num-1][0])**2 + (position[1] - self.scale*keypath[num-1][1])**2)
            # if dis < 0.2:

            #     continue

            # 计算当前采样的速度范围
            # 车子速度的最大最小范围 依次为：最小速度 最大速度 最小角速度 最大角速度速度
            Vs = [0,self.Kinematic[0],-1*self.Kinematic[1],self.Kinematic[1]]

            # 根据当前速度以及加速度限制计算的动态窗口  依次为：最小速度 最大速度 最小角速度 最大角速度
            Vd = [self.pose[3]-self.Kinematic[2]*self.dt,self.pose[3]+self.Kinematic[2]*self.dt,self.pose[4]-self.Kinematic[3]*self.dt,self.pose[4]+self.Kinematic[3]*self.dt]

            Vr = [max(Vs[0],Vd[0]),min(Vs[1],Vd[1]),max(Vs[2],Vd[2]),min(Vs[3],Vd[3])]

            evalDB = []
            # 遍历所有可能的轨迹
            for vt in np.arange(Vr[0],Vr[1],self.Kinematic[4]):
                for ot in np.arange(Vr[2],Vr[3],self.Kinematic[5]):

                    # 轨迹推测
                    time = 0
                    pose_predict = [self.pose[0],self.pose[1],self.pose[2]]
                    
                    AllPose = []
                    while time < self.evalParam[3]:
                        time = time + self.dt
                        pose_predict[0] = pose_predict[0] + self.dt*math.cos(pose_predict[2])*vt
                        pose_predict[1] = pose_predict[1] + self.dt*math.cos(pose_predict[2])*vt
                        pose_predict[2] = pose_predict[2] + self.dt*ot
                        
                        # 储存轨迹上所有点
                        AllPose.append(pose_predict)

                    # # 是否撞墙
                    # position = [int(pose_predict[0]/self.scale),int(pose_predict[1]/self.scale)]
                    # if position[0]>101 or position[1]>101 or self.map[position[0]][position[1]] == 1:
                    #     continue

                    # 评价函数
                    # 前项预测终点的航向得分  偏差越小分数越高
                    healding = self.CalcHeadingEval(pose_predict,self.goal)

                    # 轨迹上所有点距离最近障碍物的间隙得分 距离越远分数越高
                    dist = self.CalcDistEval(AllPose,vt)

                    # 速度得分 速度越快分越高
                    vel = abs(vt)

                    # # 计算制动距离
                    # stopDist = self.CalcBreakingDist(vt)

                    if dist!=0:
                        evalDB.append([vt,ot,healding,dist,vel])

            if len(evalDB) == 0:
                print('no path to goal!!')
                speed = [0,0]
                break

            evalDB = np.array(evalDB)

            # 评价函数正则化
            if np.sum(evalDB[:,2]) != 0:
                evalDB[:,2] = evalDB[:,2]/np.sum(evalDB[:,2])
            if np.sum(evalDB[:,3]) != 0:
                evalDB[:,3] = evalDB[:,3]/np.sum(evalDB[:,3])
            if np.sum(evalDB[:,4]) != 0:
                evalDB[:,4] = evalDB[:,4]/np.sum(evalDB[:,4])
            
            # 最终评价函数计算
            feval = self.evalParam[0]*evalDB[:,2] + self.evalParam[1]*evalDB[:,3] + self.evalParam[2]*evalDB[:,4]
            n = np.where(feval == np.max(feval))

            speed = [evalDB[n[0][0],0],evalDB[n[0][0],1]]
            break
        return speed 


    def CalcHeadingEval(self,pose,goal):
        theta = math.degrees(pose[2])%360
        goalTheta = math.degrees(math.atan2(goal[1]-pose[1],goal[0]-pose[0]))%360
        
        targetTheta = abs(theta - goalTheta)

        healding = 180 - targetTheta
        return healding

    def CalcDistEval(self,AllPose,vt):
        for pose_predict in AllPose:
            position = [int(pose_predict[0]/self.scale),int(pose_predict[1]/self.scale)]
            x_min = max(position[0]-self.size,0)
            y_min = max(position[1]-self.size,0)
            x_max = min(position[0]+self.size,101)
            y_max = min(position[1]+self.size,101)
            window = self.map[x_min:x_max+1,y_min:y_max+1]
            wall = np.where(window == 1)

            if np.shape(wall[0])[0] == 0:
                dis = 2.0*self.size*self.scale
            
            else:
                dis = 2000
                for i in range(np.shape(wall[0])[0]):
                    t = self.scale * math.sqrt((wall[0][i] - (self.size+1))**2 + (wall[1][i] - (self.size+1))**2)
                    stopDist = self.CalcBreakingDist(vt)
                    if t < self.obstacleR + stopDist:
                        return 0  # 分数为0

                    if dis > t:
                        dis = t
        return dis



    def CalcBreakingDist(self,vel):
        stopDist = vel**2 / (2*self.Kinematic[2])
    
        return stopDist
        

    