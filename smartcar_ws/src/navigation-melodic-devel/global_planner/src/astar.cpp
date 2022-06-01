/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>
#include "ros/ros.h"
namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

/*	function：计算规划代价的函数
	potential: 代价数组， 
	costs： 地图指针， 
	cycles：循环次数； 
*/
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));   //start_i为 地图的index，0为 代价值

    std::fill(potential, potential + ns_, POT_HIGH);  //地址potential 到 potential+ns_ 全部赋值为POT_HIGH，代价值最高
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    //目的：得到最小cost的索引，并删除它，如果索引指向goal(目的地)则退出算法，返回true
    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
            return true;

        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    //next_i 点不在网格内 忽略
    if (next_i < 0 || next_i >= ns_)
        return;

    //未搜索的点cost为POT_HIGH，如小于该值，则为已搜索点,跳过；
    if (potential[next_i] < POT_HIGH)
        return;

    //障碍物点 忽略
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential); //g(n)
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y); // h(n) 曼哈顿距离
    
    /* change */
    // int ww =  0;
    // if(distance>18) ww = 3.0;
    // else ww = 0.8;
    // queue_.push_back(Index(next_i, potential[next_i] + distance * ww * neutral_cost_));
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    /**/
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

} //end namespace global_planner
