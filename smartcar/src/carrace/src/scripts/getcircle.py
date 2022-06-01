#!/usr/bin/env python
# coding:utf-8
"""
This program is debugged by Harden Qiu
"""
from numpy import *
from scipy import optimize
import functools
from matplotlib import pyplot as p, cm, colors

# # Coordinates of the 2D points
x = r_[36, 36, 19, 18, 33, 26]
y = r_[14, 10, 28, 31, 18, 26]
# basename = 'arc'

method_2  = "Fitting Circle"
#修饰器：用于输出反馈
def countcalls(fn):
    "decorator function count function calls "

    @functools.wraps(fn)
    def wrapped(*args):
        wrapped.ncalls +=1
        return fn(*args)

    wrapped.ncalls = 0
    return wrapped

def calc_R(xc, yc):

    return sqrt((x - xc) ** 2 + (y - yc) ** 2)

@countcalls
def f_2(c):
    Ri = calc_R(*c)
    return Ri - Ri.mean()

def circle(x1,y1):
    # 质心坐标
    global x,y
    x = x1
    y = y1
    x_m = mean(x)
    y_m = mean(y)

    center_estimate = x_m, y_m
    center_2, _ = optimize.leastsq(f_2, center_estimate)

    xc_2, yc_2 = center_2
    Ri_2       = calc_R(xc_2, yc_2)
    #拟合圆的半径
    R_2        = Ri_2.mean()
    # residu_2   = sum((Ri_2 - R_2)**2)
    # residu2_2  = sum((Ri_2**2-R_2**2)**2)
    # ncalls_2   = f_2.ncalls
    L = sqrt((x[0] - x[2]) ** 2 + (y[0] - y[2]) ** 2) 
    a = [L,R_2]
    return a