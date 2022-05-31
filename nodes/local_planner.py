#!/usr/bin/env python

import numpy as np
import cv2
import global_planner
import sys
import time

class DWA_Local_Planner:
    def __init__(self, x, y, theta, my_costmap, global_path, goal, obstacle_list, sim_time, v_max, w_max, acc_max, ang_acc_max, pre_v, pre_w):

        self.x = x
        self.y = y
        self.theta = theta
        self.map = my_costmap
        self.global_path = global_path
        self.goal = goal
        self.obstacle_list = obstacle_list
       # self.h = h
       # self.f = g+h

        ## Gain
        self.path_distance_bias = 50
        self.goal_distance_bias = 0.00001
        self.occdist_scale = 0.00001

        ## Parameters
        # Simulation time, use to predict the future pose
        self.sim_time = sim_time
        # Kinematic constraint
        self.v_max = v_max
        self.v_min = 0.1
        self.w_max = w_max
        self.w_min = -w_max
        # Dynamic constraint
        self.acc_max = acc_max
        self.ang_acc_max = ang_acc_max
       # self.acc_min = -acc_max
       # self.ang_acc_min = -ang_acc_max
        # use for occdist
        self.number_of_predict = 1
        self.pre_v = pre_v
        self.pre_w = pre_w

        self.base_footprint = 7 # pixel (m)
    def cost_function(self, pose, traj):
       # pose = traj[len(traj) - 1]
       # print(pose)
        return self.path_distance_bias * self.Dist_to_path(pose) + self.occdist_scale * self.Obstacle_along_trajectory(traj) + self.goal_distance_bias * self.Dist_to_Goal(pose)
    def Get_Predict_trajectory(self, v, w):
        trajectory = []
        # This output is in the gazebo world, unit is (m, m)
        for i in range(1, self.number_of_predict + 1):
           # print(i)
            if abs(w) < 0.03:
                pose = [self.x + v * np.cos(self.theta) * self.sim_time * i / self.number_of_predict,
                        self.y + v * np.sin(self.theta) * self.sim_time * i / self.number_of_predict,
                        self.theta]
            else:
                pose = [self.x - v * np.sin(self.theta) / w + v * np.sin(self.theta + w * self.sim_time * i / self.number_of_predict) / w,
                        self.y + v * np.cos(self.theta) / w - v * np.cos(self.theta + w * self.sim_time * i / self.number_of_predict) / w,
                        self.theta + w * self.sim_time * i / self.number_of_predict]
          #  pose = [self.x - v * np.sin(self.theta) / w + v * np.sin(self.theta + w * self.sim_time) / w,
           #             self.y + v * np.cos(self.theta) / w - v * np.cos(self.theta + w * self.sim_time) / w,
           #             self.theta + w * self.sim_time * i / self.number_of_predict]

            trajectory.append(pose)
            print(v, w, pose)
       # print("Get predict:", pose, v, w)
        return trajectory[len(trajectory) - 1], trajectory
    def Find_action(self):
        # time_start = time.time()
        Minimum_cost = sys.float_info.max
#        for v in np.arange(max(self.pre_v - self.acc_max, self.v_min), min(self.v_max, self.pre_v + self.acc_max), 0.02):
        for v in np.arange(0.1, 0.4, 0.03):

            for w in np.arange(-0.4, 0.4, 0.02):
                pose, traj = self.Get_Predict_trajectory(v, w)
               # print(v, w)
                if self.cost_function(pose, traj) < Minimum_cost:
                    Minimum_cost = self.cost_function(pose, traj)
                    action_list = [v, w]
       # print(self.x, self.y)
       # self.Update_pre_v_and_w(action_list[0], action_list[1])
       # print(action_list)
       # time_end = time.time()
       # time_c = time_end - time_start
       # print('time cost', time_c, 's')

        return action_list
    def Update(self, x, y, theta, v, w):

        self.x = x
        self.y = y
        self.theta = theta
        self.pre_v = v
        self.pre_w = w
    def Dist_to_Goal(self, pose):
     #   return 0
        return np.sqrt((pose[0] - self.goal[0]) * (pose[0] - self.goal[0]) + (pose[1] - self.goal[1]) * (pose[1] - self.goal[1]))
    def Dist_to_path(self, pose):
        Minimum_dist = sys.float_info.max

        for path_grid in self.global_path:
            # Find closest distance
           # x, y = T_map_to_gazebo(path_grid[0], path_grid[1])
           # print("path:", x, y)
            dist = (pose[0] - path_grid[0]) * (pose[0] - path_grid[0]) + (pose[1] - path_grid[1]) * (pose[1] - path_grid[1])
            if dist < Minimum_dist:
                Minimum_dist = dist
                rec_path = path_grid
        print("predict:", rec_path)
        return np.sqrt(Minimum_dist)
    def Obstacle_along_trajectory(self, traj):
        how_much_obstacle = 0
        for pose in traj:
            xi = int(pose[0] * 204 / 11 + 11602 / 55)  # xi, yi, ti are transform to map coordinate
            yi = int(pose[1] * -1200 / 73 + 12708 / 73)
            for x in range(xi - self.base_footprint, xi + self.base_footprint + 1):
                for y in range(yi - self.base_footprint, yi + self.base_footprint + 1):
                    if (x, y) in self.obstacle_list:
                       # print("obs", x, y)
                        how_much_obstacle = how_much_obstacle + 1

        # for obs in self.obstacle_list:
       #     for pose in traj:
       #         if (obs[0] - pose[0]) * (obs[0] - pose[0]) + (obs[1] - pose[1]) * (obs[1] - pose[1]) < self.base_footprint * self.base_footprint:
       #             how_much_obstacle = how_much_obstacle + 1

        # print(how_much_obstacle)
        return how_much_obstacle
def T_map_to_gazebo(x, y):
    return 11 * x / 204 - 5801 / 510, -14.6 * y / 240 + 10.59
  #  def __lt__(self, other):
  #      return self.f < other.f
  #  def __gt__(self, other):
  #      return self.f > other.f
  #  def __eq__(self, other):
  #      return self.f == other.f
        

