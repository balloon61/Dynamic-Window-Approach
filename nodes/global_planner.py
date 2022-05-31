#!/usr/bin/env python
import argparse

import cv2
import numpy as np
import queue
import heapq
import local_planner
import matplotlib.pyplot as plt
import time

import rospy
from geometry_msgs.msg import Twist, Pose, Point
import sys, select, os
from astar_search import Astar
if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from scipy.signal import convolve2d

my_map = []
pq = queue.PriorityQueue()
initial_input = []
goal_input = []

position = [(0.0, 0.0, 0.0)]
orientation = [0.0]
def callback(data):
#    print(data.pose)
#    print("!", data.pose[0])
#    print("2", data.pose[1])
#    print("3", data.pose[2])

    bot_x = data.pose[-1].position.x
    bot_y = data.pose[-1].position.y
    orien = data.pose[-1].orientation
    g_orien = data.pose[-2].orientation
    _, _, yaw = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    position[0] = (bot_x, bot_y, yaw)
    _,_,goal_theta = euler_from_quaternion([g_orien.x, g_orien.y, g_orien.z, g_orien.w])

def callback1(msg):

    orien = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
    position[0] = msg.pose.pose.position.x
    position[1] = msg.pose.pose.position.y
    position[2] = yaw

def odometryCb(msg):

    global roll, pitch, yaw, gx, gy, gz
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    (gx, gy, gz) = msg.pose.pose

# my data structure to record node
class Node:
    def __init__(self, current_pose, parents, index, cost):
        self.pose = current_pose
      #  self.goal = goal_pose
        self.parents = parents
        self.child = []
        self.index = index
        self.cost = cost


    def __lt__(self, other):
        return self.cost < other.cost

    # a function to dynamically add children
    def add_child(self, child):
        self.child.append(child)
        return

class CMap:
    def __init__(self, value, color):
        self.value = value
        self.color = color

    def change(self, x, y, value, color):
        self.value[x][y] = value
        self.color[x][y] = color

    def check_obstacle_color(self):
        for i in range(len(self.value)):
            for j in range(len(self.value[0])):
                if self.value[i][j] <= 0:
                    self.color[i][j] = np.array([0, 0, 0])
        # print(self.value)


# -1 obstacle, 0 unvisited, 1 visited, 2 initial pose, 3 goal pose, 254 in queue but have not visited yet


# Global planner A*

def A_Star_Global_Planner(First):  # Dijkstra algorithm
    # eight direction to explore
    direction = np.array([[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [-1, -1], [1, -1], [-1, 1]])
    pq = []
    # create a priority queue
    heapq.heappush(pq, First)
    #    pq.put((First.cost, First))

    while len(pq) >= 0:
        # get the minimum cost pose
        current = heapq.heappop(pq)
        my_map.change(current.pose[0], current.pose[1], 1, (255, 0, 0))
        # 8 direction
        for i in range(len(direction)):

            # if not visited yet, put it into the queue
            if bw_map[current.pose[0] + direction[i][0]][current.pose[1] + direction[i][1]] == 255: # 255 (white) means can achieved and unvisited
                bw_map[current.pose[0] + direction[i][0]][current.pose[1] + direction[i][1]] = 254  # change it to visited
                my_map.change(current.pose[0] + direction[i][0], current.pose[1] + direction[i][1], 254, (255, 0, 0))
                Explore_Node = Node(np.array([current.pose[0] + direction[i][0], current.pose[1] + direction[i][1]]),
                                    current, current.index + 1, current.cost + np.linalg.norm(direction[i]) + dist_to_goal(np.array([current.pose[0] + direction[i][0], current.pose[1] + direction[i][1]])) - dist_to_goal(np.array([current.pose[0], current.pose[1]])))
                current.add_child(Explore_Node)
                heapq.heappush(pq, Explore_Node)
               #     Show_image()
            # if it is equal to goalm then return it
            elif my_map.value[current.pose[0] + direction[i][0]][current.pose[1] + direction[i][1]] == 3:
                return Node(np.array([current.pose[0] + direction[i][0], current.pose[1] + direction[i][1]]), current,
                            current.index + 1, current.cost + np.linalg.norm(direction[i]))
                # pq.put((cost, Node(np.array([current[0] + direction[i][0], current[1] + direction[i][1]]), current,
                #                   current.index + 1, cost)))
    return False

def dist_to_goal(current_pose):
    return 0
    #return np.linalg.norm((goal_input[0] - current_pose[0]) * (goal_input[0] - current_pose[0]) + (goal_input[1] - current_pose[1]) * (goal_input[1] - current_pose[1]))

def Find_path(Final_node):
    # backtracking, from the goal to the initial via parents
    global_path = []
    while Final_node.pose[0] != int(initial_input[0]) or Final_node.pose[1] != int(initial_input[1]):
        global_path.append(Final_node.pose)
        # print(Final_node.pose)
        my_map.change(Final_node.pose[0], Final_node.pose[1], 4, (0, 0, 255))
        Final_node = Final_node.parents
    return global_path


def Show_image():
    # Original image
    cv2.imshow("map", my_map.color)
    # Rotate image
    #    cv2.imshow("map", cv2.flip(my_map.color, 0))
    cv2.waitKey(1)
def create_obstacle_list(map):
    rows, cols = map.shape
    obs_dict = {}
    for i in range(90, 345):
        for j in range(205, 435):
            if map[i, j] == 0:
                #obs_list.append([i, j])
                obs_dict[(j, i)] = True
               # my_map.change(i, j, 4, (255, 0, 255))

    return obs_dict
if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--start', nargs='+', type=float, required=True)
    parser.add_argument('-g', '--goal', nargs='+', type=float, required=True)
   # parser.add_argument('-rpm', '--RPM', nargs='+', type=str)
   # parser.add_argument('-c', '--clearance', type=float, default=0.1)
   # parser.add_argument('-f', '--file_name', type=str, default="visualize")
    args = parser.parse_args()

    rospy.init_node('turtlebot_teleop')

    # publisher
    pub_linear_slide = rospy.Publisher('/mmrobot/linear_joint/command', Float64, queue_size=10)

    pub_left_arm_j1 = rospy.Publisher('/mmrobot/l_arm_joint1/command', Float64, queue_size=10)
    pub_left_arm_j2 = rospy.Publisher('/mmrobot/l_arm_joint2/command', Float64, queue_size=10)
    pub_left_arm_j3 = rospy.Publisher('/mmrobot/l_arm_joint3/command', Float64, queue_size=10)
    pub_left_gripper = rospy.Publisher('/mmrobot/l_gripper_joint/command', Float64, queue_size=10)

    pub_right_arm_j1 = rospy.Publisher('/mmrobot/r_arm_joint1/command', Float64, queue_size=10)
    pub_right_arm_j2 = rospy.Publisher('/mmrobot/r_arm_joint2/command', Float64, queue_size=10)
    pub_right_arm_j3 = rospy.Publisher('/mmrobot/r_arm_joint3/command', Float64, queue_size=10)
    pub_right_gripper = rospy.Publisher('/mmrobot/r_gripper_joint1/command', Float64, queue_size=10)

    pub_mobile = rospy.Publisher('/mmrobot/mobile_controller/cmd_vel', Twist, queue_size=10)
    # sub_odom = rospy.Subscriber('/mmrobot/mobile_controller/odom', Odometry, callback1)
    # sub_odom = rospy.Subscriber('/mmrobot/mobile_controller/odom', Odometry, callback)
    #  rospy.init_node('oodometry', anonymous=True) #make node

    sub_odom = rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    # Transformation from Gazebo world to image
    r = rospy.Rate(1)

    xi = args.start[0] * 204 / 11 + 11602/55 # xi, yi, ti are transform to map coordinate
    yi = args.start[1] * -1200 / 73 + 12708 / 73
    ti = args.start[2] * 180 / np.pi

    xg = args.goal[0] * 204 / 11 + 11602/55
    yg = args.goal[1] * -1200 / 73 + 12708 / 73

    initial_input = [yi, xi]
    goal_input = [yg, xg]
    # define the initial and goal's (x, y)
    # initialize parameters

    # clearance_map = convolve2d(OMap, kernel, mode='same')
    map = cv2.imread("my_map.pgm")
    gray = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
    thresh, bw_map = cv2.threshold(gray, 222, 255, cv2.THRESH_BINARY)
    thresh, save_bw = cv2.threshold(gray, 222, 255, cv2.THRESH_BINARY)

    kernel = np.ones((15, 15), np.uint8)
    kernel2 = np.ones((5, 5), np.uint8)

    bw_map = cv2.erode(bw_map, kernel, iterations=1)
    save_bw = cv2.erode(save_bw, kernel, iterations=1)
    h, w, c = map.shape
    my_map = CMap(bw_map, map)  # value, color
    # my_map.check_obstacle_color()
    # Visualize initial pose and final pose
    print(int(yi), int(xi))
    print(int(yg), int(xg))

    my_map.change(int(yi), int(xi), 2, (255, 0, 0)) # 2 = initial point
    my_map.change(int(yg), int(xg), 3, (0, 0, 255)) # 3 = goal point
    bw_map[int(yg)][int(xg)] = 3

    # Dijkstra and backtracking
    global_path = Find_path(A_Star_Global_Planner(Node(np.array([int(yi), int(xi)]), None, 0, dist_to_goal(np.array([int(yg), int(xg)])))))  # pose, parents, index, cost
   # print(global_path)
   # cv2.imshow("map", my_map.color)
    #  cv2.imshow("gray", gray)

    #cv2.waitKey(0)
    obstacle_list = create_obstacle_list(save_bw)
    #print(obstacle_list)
   # print(len(obstacle_list))
   # cv2.imshow("map", my_map.color)
    global_path_with_meter = []
    px = []
    py = []
    #     def __init__(self, x, y, theta, my_costmap, global_path, goal, obstacle_list, sim_time, v_max, w_max, acc_max, ang_acc_max, pre_v, pre_w):
    for gp in global_path:
        x_new, y_new = local_planner.T_map_to_gazebo(gp[1], gp[0])
        global_path_with_meter.append([x_new, y_new])
        px.append(x_new)
        py.append(y_new)
        print(x_new, y_new)
    dwa = local_planner.DWA_Local_Planner(args.start[0], args.start[1], args.start[2], # Start(x, y, t) unit: m, m, radian
                                          save_bw, # binary map
                                          global_path_with_meter, # unit pixel
                                          [args.goal[0], args.goal[1]], # goal (x, y) unit are (m, m)
                                          obstacle_list, # all obstacle position
                                          4, # simulation time
                                          0.4, 0.4, # v max, w max
                                          0.2, 0.2, # a max, ang a max
                                          0, 0) # previous v, w
    estimate_x = args.start[0]
    estimate_y = args.start[1]
    estimate_t = args.start[2]
    Low_level_gain = 2.03811951707
    Workspace = 0.6
    time_interval = 0.5
    number_of_divide_point = 10
    twist = Twist()
   # v, w = dwa.Find_action()
   # cv2.imshow("map", save_bw)

  #  x = position[-1][0]
  #  y = position[-1][1]
 #   th = position[-1][2]
    # print(x, y, th)
  #  dwa.Update(x, y, th, 0, 0)
  #  v, w = dwa.Find_action()

  #  print(v, w)
   # cv2.imshow("gray", my_map.color)
    Gx = []
    Gy = []
    Gt = []
    rec_time = []
    rec_v = []
    rec_w = []
    time_start = time.time()

    #  cv2.waitKey(0)
    while (estimate_x - args.goal[0]) * (estimate_x - args.goal[0]) + (estimate_y - args.goal[1]) * (estimate_y - args.goal[1]) > Workspace * Workspace:

        v, w = dwa.Find_action()
        #for time in np.arange(0, time_interval + time_interval / number_of_divide_point, time_interval / number_of_divide_point):
        #    estimate_x = estimate_x - v * np.sin(estimate_t) / w + v * np.sin(estimate_t + w * time) / w
        #    estimate_y = estimate_y - v * np.cos(estimate_t) / w - v * np.cos(estimate_t + w * time) / w
        #    estimate_t = estimate_t + w * time
        # if abs(w) < 0.1:
        #     estimate_x = estimate_x + v * np.sin(estimate_t)
        #     estimate_y = estimate_y + v * np.cos(estimate_t)
        # else:
        #     estimate_x = estimate_x - v * np.sin(estimate_t) / w + v * np.sin(estimate_t + w * time_interval) / w
        #     estimate_y = estimate_y - v * np.cos(estimate_t) / w - v * np.cos(estimate_t + w * time_interval) / w
        #     estimate_t = estimate_t + w * time_interval
        # print("pose:", estimate_x, estimate_y, estimate_t)
        print("kinematic", v, w)
        twist.linear.x = v * Low_level_gain
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = w * Low_level_gain

        #  print('(v, w)', '(', control_linear_vel, ',', control_angular_vel, ')')
        pub_mobile.publish(twist)
       # r.sleep()
        x = position[-1][0]
        y = position[-1][1]
        th = position[-1][2]
        estimate_x = position[-1][0]
        estimate_y = position[-1][1]
        estimate_t = position[-1][2]
       # print(x, y, th)
        Gx.append(x)
        Gy.append(y)
        dwa.Update(x, y, th, v, w)
        time_end = time.time()
        time_c = time_end - time_start
        rec_time.append(time_c)
        rec_v.append(v)
        rec_w.append(w)
 #   plt.plot(Gx, Gy, 'g', label='Actual path')
 #   plt.plot(px, py, 'r', label='Global path')

#    plt.xlabel('x (m)')
#    plt.ylabel('y (m)')
#    plt.title('Actual oath vs Global path')
#    plt.grid(True)
#    plt.legend(loc='best')

#    plt.xlim([0, 12])
#    plt.ylim([-10, 2])
#    plt.show()

    plt.plot(rec_time, rec_v, 'r')

    plt.xlabel('t (sec)')
    plt.ylabel('v (m/s)')
    plt.title('v-t')
    plt.grid(True)
    plt.xlim([0, 55])
    plt.ylim([0, 0.5])
    plt.show()
    plt.plot(rec_time, rec_w, 'r')

    plt.xlabel('t (sec)')
    plt.ylabel('w (rad/s)')
    plt.title('w-t')
    plt.grid(True)
    plt.xlim([0, 55])
    plt.ylim([-0.5, 0.5])
    plt.show()
#   Show_image()
#    cv2.waitKey(0)






    # clearance 5mm


""" 
    # Show_image()

    # if have solution then visuliza
    if have_ans:
        OMap = create_map(np.zeros((250, 400), dtype=int))
        my_map = CMap(OMap, np.ones((250, 400, 3), np.uint8) * 255)  # value, color
        my_map.check_obstacle_color()
        clearance_map = convolve2d(OMap, kernel, mode='same')
        my_map.change(I_input[0], I_input[1], 2, (255, 0, 0))
        my_map.change(G_input[0], G_input[1], 3, (0, 0, 255))
        clearance_map[G_input[0]][G_input[1]] = 3
        Find_path(Dijkstra(Node(np.array([I_input[0], I_input[1]]), None, 0, 0), 1), 1)  # pose, parents, index, cost
        Show_image()
        cv2.waitKey(0)
        


    

    I_input = []
    G_input = []
    


    # input the initial and goal's (x, y), if the pose is occupied or too close to the obstacle then input again
    initial_input = input("Enter initial (x,y):\n").split()
    initial_input = [int(initial_input[0]), int(initial_input[1])]
    if bw_map[initial_input[0]][initial_input[1]] != 0:
        I_input.append(int(initial_input[0]))
        I_input.append(int(initial_input[1]))
    else:
        while bw_map[initial_input[0]][initial_input[1]] != 0:
            initial_input = input("ERROR, initial is in obstacle, Please Enter initial (x, y) again\n").split()
            initial_input = [int(initial_input[0]), int(initial_input[1])]
            if bw_map[initial_input[0]][initial_input[1]] > -1:
                I_input.append(int(initial_input[0]))
                I_input.append(int(initial_input[1]))
    goal_input = input("Enter goal (x,y):\n").split()
    goal_input = [int(goal_input[0]), int(goal_input[1])]
    if bw_map[goal_input[0]][goal_input[1]] != 0:
        G_input.append(int(goal_input[0]))
        G_input.append(int(goal_input[1]))
    else:
        while bw_map[goal_input[0]][goal_input[1]] != 0:
            goal_input = input("ERROR, goal is in obstacle, Please Enter initial (x, y) again\n").split()
            goal_input = [int(goal_input[0]), int(goal_input[1])]
            if bw_map[goal_input[0]][goal_input[1]] != 0:
                G_input.append(int(goal_input[0]))
                G_input.append(int(goal_input[1]))
"""