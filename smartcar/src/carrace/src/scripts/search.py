#!/usr/bin/env python
# coding:utf-8
import queue
import math
import numpy as np
import cv2
import copy

class PathPlanning():
    def __init__(self,start,end):
        self.dirs = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        # self.h = []
        # self.h.append((0, 0))
        # self.h.append((0, 1))

        # 出发点和终点
        self.start = start
        self.goal = end

        # 地图
        # self.Map = map

        # 自建地图,测试用
        self.Map = self.creatmap()
        self.Mapcopy = copy.deepcopy(self.Map)
        self.bigger(5)

    def path(self):
        # 外部输入时用
        # Map = self.map
        # Map = self.bigger(5)
        
        frontier = queue.PriorityQueue()
        frontier.put(self.start, 0)
        visited = {}
        visited[self.start] = True
        came_from = {}
        cost_so_far = {}
        came_from[self.start] = None
        cost_so_far[self.start] = 0

        while not frontier.empty():
            current = frontier.get()
            for next in self.neighbors(current):
                if not len(self.neighbors(current)):
                    break
                if next not in visited:
                    frontier.put(next)
                    visited[next] = True

            if current == self.goal:
                break

            if next not in visited:
                frontier.put(next)
                visited[next] = True
            for next in self.neighbors(current):

                new_cost = cost_so_far[current] + 1
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(self.goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        current = self.goal
        path = []
        while current != self.start:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        path.reverse()
        keypath = self.keypoint(path)
        keypath = self.keypoint2(keypath)
        return keypath


    def creatmap(self):
        map = np.zeros([100,100],dtype=int)
        a = np.ones([20,20])   # 
        map[60:80,0:20] = a

        a = np.ones([30,40])
        map[70:100,40:80] = a

        a = np.ones([10,60])
        map[20:30,0:60] = a

        a = np.ones([30,20])
        map[20:50,80:100] = a
        return map

    # 障碍物膨胀
    def bigger(self,n):
        # 墙往外扩展5个点,障碍物膨胀
        self.Map = np.array(self.Map,dtype='uint8')
        self.Map = np.where(self.Map>0,255,self.Map)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)) #定义结构元素的形状和大小
        self.Map = cv2.dilate(self.Map, kernel,iterations=n)#膨胀操作

        # 查看图片效果
        # cv2.imshow('1',self.Map)
        # cv2.waitKey(0)
        self.Map = np.where(self.Map>0,1,self.Map)
        self.Map = self.Map.tolist()


    def neighbors(self,current):
        neib = []
        for i in range(4):
            if 0 <= current[0] + self.dirs[i][0] < 100 and 0 <= current[1] + self.dirs[i][1] < 100:
                if self.Map[current[0] + self.dirs[i][0]][current[1] + self.dirs[i][1]] == 0:
                    neib.append((current[0] + self.dirs[i][0], current[1] + self.dirs[i][1]))
        if not len(neib):
            return []
        return neib


    def heuristic(self,pos,end):
        c = abs(end[0] - pos[0]) + abs(end[1] - pos[1])
        return c

    # 去除距离较近的关键点
    def keypoint2(self,keypath):
        num = -2
        flag = 0
        while(1):
            dis = abs(keypath[num-1][0] - keypath[num][0]) + abs(keypath[num-1][1] - keypath[num][1])
            if dis < 10:
                up = np.array(keypath[num+1]) - np.array(keypath[num])
                down = np.array(keypath[num-1]) - np.array(keypath[num])
                up_ = np.where(up != 0)[0][0]
                down_ = np.where(down != 0)[0][0]

                if up_ == 0:
                    newpoint = (keypath[num+1][up_],keypath[num-1][down_])
                    min1 = min(keypath[num-1][up_],newpoint[up_])
                    max1 = max(keypath[num-1][up_],newpoint[up_])
                    for i in range(min1,max1+1):
                        if self.Map[i][keypath[num-1][down_]] == 1:
                            flag = 1
                            break
                    min2 = min(newpoint[down_],keypath[num+1][down_])
                    max2 = max(newpoint[down_],keypath[num+1][down_])
                    for j in range(min2,max2+1):
                        if self.Map[keypath[num+1][up_]][j] == 1:
                            flag = 1
                            break

                else:
                    newpoint = (keypath[num-1][down_],keypath[num+1][up_])
                    min1 = min(keypath[num-1][up_],newpoint[up_])
                    max1 = max(keypath[num-1][up_],newpoint[up_])
                    for i in range(min1,max1):
                        if flag == 0 and self.Map[keypath[num-1][down_]][i] == 1:
                            flag = 1
                            break
                    min2 = min(newpoint[down_],keypath[num+1][down_])
                    max2 = max(newpoint[down_],keypath[num+1][down_])
                    for j in range(min2,max2):
                        if flag == 0 and self.Map[j][keypath[num+1][up_]] == 1:
                            flag = 1
                            break
                
                if flag == 0:
                    keypath[num] = newpoint
                    keypath.remove(keypath[num+1])
                    num = num + 1


            num = num - 1
            if abs(num) == len(keypath):
                break

        return keypath


    def keypoint(self,path):
        keypath = []
        a = np.array([True,True])
        for i in range(len(path)-1):
            now = np.array(path[i])
            next = np.array(path[i+1])
            x = now == next
            if (a == x).all():
                continue
            else:
                keypath.append(path[i])
                a = x
        if keypath[-1] != path[-1]:
            keypath.append(path[-1])
        return keypath

