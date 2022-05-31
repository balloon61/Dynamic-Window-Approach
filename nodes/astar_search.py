#!/usr/bin/env python

import numpy as np
import cv2
import heapq
from tqdm import tqdm
import argparse
from util import Circle, Polygon_Obstacle

class Node:
    def __init__(self, x, y, theta, g, h, hist_traj, robot_param):
        self.x = x
        self.y = y
        self.theta = theta
        self.g = g
        self.h = h
        self.f = g+h
        self.hist_traj = hist_traj
        self.robot_param = robot_param
    def __lt__(self, other):
        return self.f < other.f
    def __gt__(self, other):
        return self.f > other.f
    def __eq__(self, other):
        return self.f == other.f
        

# Main class to perform A* search on the defined map
class Astar:
    def __init__(self, m, n, clearance, robot_l, wheel_radius, time_step, thres, RPM, resolution=(0.5, 30), video_resolution=0.01):
        self.clearance = clearance
        self.radius = robot_l/2
        self.thres = thres
        self.resolution = resolution

        # Construct Obstacles
        self.obstacle_list = []
        self.createObstacleSpace()

        self.m = m
        self.n = n
        self.video_resolution = video_resolution

        self.reset() # Initialize data structures for searching

        self.drawObstacles()

        self.wheel_radius = wheel_radius
        self.L = robot_l
        self.dt = 0.1
        self.time_step = time_step

        # Convert 2 rpms to into 8 actions (rad/s)
        rpm1, rpm2 = RPM
        self.set_action_list(rpm1, rpm2)

    def reset(self):
        m = self.m
        n = self.n
        resolution = self.resolution
        video_resolution = self.video_resolution

        self.Nodes = []
        self.visited = np.zeros([int((m+1)/resolution[0]),
                                 int((n+1)/resolution[0]),
                                 int(360/resolution[1])+1])
        self.map = np.zeros([int(m/video_resolution),int(n/video_resolution),3])+0.1
        self.path = []

    def set_action_list(self, rpm1, rpm2):
        def rpm2rad(rpm): return rpm*np.pi/30
        self.action_list = []
        self.action_list.append([0, rpm2rad(rpm1)]) 
        self.action_list.append([rpm2rad(rpm1),0]) 
        self.action_list.append([rpm2rad(rpm1),rpm2rad(rpm1)]) 
        self.action_list.append([0, rpm2rad(rpm2)]) 
        self.action_list.append([rpm2rad(rpm2),0]) 
        self.action_list.append([rpm2rad(rpm2),rpm2rad(rpm2)]) 
        self.action_list.append([rpm2rad(rpm1),rpm2rad(rpm2)]) 
        self.action_list.append([rpm2rad(rpm2),rpm2rad(rpm1)]) 

    def set_start_goal(self, start, goal):
        s_x,s_y,s_t = start

        if s_t < 0: s_t += 360
        if s_t > 0: s_t %= 360

        g_x,g_y = goal

        self.start = start
        self.goal = goal

        # Check if start and goal positions are valid
        if not self.valid_loc(s_x,s_y,s_t) or not self.valid_loc(g_x,g_y,0): 
            print('Invalid input, drawing...')
            self.drawValidArea()
            self.draw_start_goal()
            
            print('Make sure the start and goal positions are in the green area.')
            cv2.imshow('Invalid input', self.map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            raise Exception("Invalid start position and goal position")

    # Draw start and goal positions
    def draw_start_goal(self):
        r = self.video_resolution

        l = 10
        n = self.map.shape[0]
        x1 = int(self.start[0]/r) 
        y1 = int(n-self.start[1]/r)
        gx1 = int(self.goal[0]/r)
        gy1 = int(n-self.goal[1]/r)
         
        x2 = int(x1 + l*np.cos(np.pi/180*self.start[2]))
        y2 = int(y1 - l*np.sin(np.pi/180*self.start[2]))

        cv2.circle(self.map, (x1,y1), 3, (0,0,255), 2)
        cv2.circle(self.map, (gx1,gy1), 3, (0,0,255), 2)
        cv2.line(self.map, (x1,y1), (x2,y2), (0,0,255), 2)

    def drawObstacles(self):
        y = self.map.shape[0]
        r = self.video_resolution
        clr = (255,0,0)
        
        square1 = np.array([[1.75/r,y-4.25/r],
                            [1.75/r,y-5.75/r],
                            [0.25/r,y-5.75/r],
                            [0.25/r,y-4.25/r]]).astype(int)

        square2 = np.array([[6.25/r,y-4.25/r],
                            [6.25/r,y-5.75/r],
                            [3.75/r,y-5.75/r],
                            [3.75/r,y-4.25/r]]).astype(int)

        square3 = np.array([[8.75/r,y-2/r],
                            [8.75/r,y-4/r],
                            [7.25/r,y-4/r],
                            [7.25/r,y-2/r]]).astype(int)

        cv2.polylines(self.map, [square1, square2, square3], 
                        True, clr, 2)

        cv2.circle(self.map, (int(2/r),int(y-2/r)), int(1/r), (255,0,0), 2)
        cv2.circle(self.map, (int(2/r),int(y-8/r)), int(1/r), (255,0,0), 2)
    
    # Use when input is invalid
    def drawValidArea(self):
        m,n,_ = self.map.shape
        r = self.video_resolution
        for i in range(m):
            for j in range(n):
                if self.valid_loc(j*r,i*r,0): 
                    self.map[m-i,j] = np.array([0,150,0])
                

    # Construct Obstacle Spaces
    def createObstacleSpace(self):
        P1 = Polygon_Obstacle()
        P1.construct([((1.75,4.25),(1.75,5.75),False),
                      ((1.75,5.75),(0.25,5.75),False),
                      ((0.25,5.75),(0.25,4.25),True),  
                      ((0.25,4.25),(1.75,4.25),True)])
        self.obstacle_list.append(P1)

        P2 = Polygon_Obstacle()
        P2.construct([((6.25,4.25),(6.25,5.75),False),
                      ((6.25,5.75),(3.75,5.75),False),
                      ((3.75,5.75),(3.75,4.25),True),  
                      ((3.75,4.25),(6.25,4.25),True)])
        self.obstacle_list.append(P2)

        P3 = Polygon_Obstacle()
        P3.construct([((8.75,2),(8.75,4),False),
                      ((8.75,4),(7.25,4),False),
                      ((7.25,4),(7.25,2),True),   
                      ((7.25,2),(8.75,2),True)])
        self.obstacle_list.append(P3)
        self.obstacle_list.append(Circle((2,2),1))
        self.obstacle_list.append(Circle((2,8),1))

    # Calculate heuristic based on (DISTANCE TO GOAL)/(STEP SIZE)
    def heuristic(self,x,y,theta):
        theta_r = self.resolution[1]
        #theta_d = np.abs(self.goal[2]-theta)
        r_d = ((self.goal[0]-x)**2 + (self.goal[1]-y)**2)**0.5 - self.thres
        return np.maximum(r_d, 0)
        
    def create_traj(self,x,y,theta,ul,ur):
        t = 0
        r = self.wheel_radius
        L = self.L
        dt = self.dt
        traj = [(x,y)]
        next_x = x
        next_y = y
        cost = 0

        theta_rad = np.deg2rad(theta)

        v = 0.5*r*(ul+ur)
        w = (r/L) * (ur - ul)
        while t < self.time_step:
            dx = v*np.cos(theta_rad)*dt 
            dy = v*np.sin(theta_rad)*dt 
            x += dx
            y += dy
            theta_rad += w * dt
            traj.append([x,y])
            t += dt
            cost += (dx**2 + dy**2)**0.5

        theta = np.rad2deg(theta_rad)
        while theta < 0:
            theta += 360
        if theta > 360:
            theta %= 360

        table = {'next_state': (x, y, theta),
                 'traj': traj, 
                 'cost': cost,
                 'robot': (v,w)}
        return table
    # Construct next position lists 
    # Next positions must be valid to be put in the list
    def nextNodes(self, node):
        nextList = []

        for ul,ur in self.action_list:
            next_config = self.create_traj(node.x,node.y,node.theta,ul,ur)
            next_x,next_y,next_theta = next_config['next_state']
            traj = next_config['traj']
            cost = next_config['cost']
            robot_param = next_config['robot']

            #print(next_x, next_y, next_theta)
            #print(next_x,next_y,next_theta)
            if not self.valid_loc(next_x,next_y,next_theta): continue
            #else: print('valid')
            # Make sure trajectory is valid

            for x,y in traj:
                #print(x,y)
                if not self.valid_loc(x,y,0): break
            else:
                next_g = node.g + cost
                next_h = self.heuristic(next_x,next_y,next_theta)
                nextNode = Node(next_x,next_y,next_theta,next_g,next_h,traj,robot_param)
                nextList.append(nextNode)

        return nextList
   
    # Check if the location is a valid location: Not in obstacle or clearance area or visited
    def valid_loc(self,x,y,theta):
        total_clearance = self.clearance+self.radius
        if x < total_clearance or\
           x >= self.m-total_clearance or\
           y < total_clearance or\
           y >= self.n-total_clearance: return False

        grid_x = x/self.resolution[0]
        grid_y = y/self.resolution[0]
        grid_x = int(grid_x) + int((np.ceil(grid_x) - grid_x) < 0.5)
        grid_y = int(grid_y) + int((np.ceil(grid_y) - grid_y) < 0.5)

        grid_t = int(theta/self.resolution[1])

        if self.visited[grid_x,grid_y,grid_t] == 1: return False

        for obstacle in self.obstacle_list:
            inObstacle = True
            inObstacle = inObstacle and obstacle.inside((x,y),total_clearance)
            if inObstacle:
                return False
        
        return True

    def markVisited(self, node):
        x = node.x
        y = node.y
        theta = node.theta

        #grid_x = int(x) + int((np.ceil(x) - x) < 0.5)
        #grid_y = int(y) + int((np.ceil(y) - y) < 0.5)
        grid_x = x/self.resolution[0]
        grid_y = y/self.resolution[0]
        grid_x = int(grid_x) + int((np.ceil(grid_x) - grid_x) < 0.5)
        grid_y = int(grid_y) + int((np.ceil(grid_y) - grid_y) < 0.5)

        #grid_x = int(grid_x/self.resolution[0])
        #grid_y = int(grid_y/self.resolution[0])
        grid_t = int(theta/self.resolution[1])
        self.visited[grid_x,grid_y,grid_t] = 1
    
    # Check if current position is close to the goal within a certain threshold
    def reachGoal(self, node):
        d = (self.goal[0]-node.x)**2+(self.goal[1]-node.y)**2
        return d <= self.thres**2 
        
    
    # Main search algorithm
    def search(self):
        x,y,theta = self.start
        startNode = Node(x,y,theta,0,self.heuristic(x,y,theta),[],None)
        # Mark start node as visited
        self.Nodes.append((startNode, -1))

        #self.markVisited(startNode)

        # Initialize Priority Queue with the start node
        pq = [(startNode, 0)]
        reach = False
        while(len(pq) > 0 and not reach):
            #print(len(self.Nodes))
            # Pop the node with the smallest cost
            curNode, idx = heapq.heappop(pq)
            #print(curNode.x, curNode.y)
            # Get next valid positions
            nextNodes = self.nextNodes(curNode)
            for node in nextNodes:
                # Mark next nodes as visited
                self.markVisited(node)
                self.Nodes.append((node, idx))
                # Push next nodes to priority queue 
                heapq.heappush(pq, (node, len(self.Nodes)-1))

                # Terminate if reach goal
                if self.reachGoal(node): 
                    reach = True
                    idx = len(self.Nodes)-1
                    break
        if not reach:
            self.draw_start_goal()

            #---------Debug-----------
            #visited = self.visited.sum(-1) > 0
            #visited = visited.astype(np.uint8)*255
            #cv2.imshow('test', visited)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
            
            cv2.imshow('No path found', self.map)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            #raise Exception("No path found")
                        
        # Backtrack to find the path
        else:
            self.generate_path(idx)
            self.path = self.path[::-1]

    # Recursive function that finds the path from goal to start
    def generate_path(self, cur_idx):
        if cur_idx == -1: return

        node, parent_idx = self.Nodes[cur_idx]
        self.path.append(node)
        self.generate_path(parent_idx)

    def get_robot_actions(self):
        robot_actions = []
        for node in self.path:
            if node.robot_param is None: continue
            v, w = node.robot_param
            x = node.x
            y = node.y
            theta = node.theta
            robot_actions.append((v,w,x,y,theta))
        return robot_actions
            
            
    # Store search process to video
    def visualize(self, file_name):
        y,x,_ = self.map.shape
        r = self.video_resolution
        self.draw_start_goal()

        cap = cv2.VideoCapture(0)
        fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        out = cv2.VideoWriter(file_name, fourcc, 60.0, (x,y))
        out.write(self.map.astype(np.uint8))

        for i, (node, idx) in enumerate(tqdm(self.Nodes)):
            if idx == -1: continue
            parent_node, _ = self.Nodes[idx]
            if len(node.hist_traj) > 0:
                traj = np.array(node.hist_traj)
                traj[:,0] /= r
                traj[:,1] = y - traj[:,1]/r
                traj = traj.astype(np.int32)
                cv2.polylines(self.map, [traj.astype(int)], False, (255,255,255),1)
            #cv2.line(self.map, (int(node.x/r), int(y-node.y/r)),\
            #            (int(parent_node.x/r), int(y-parent_node.y/r)),(255,255,255),1)
            if i % 50 == 0:
                self.draw_start_goal()
                out.write(self.map.astype(np.uint8))

        self.draw_start_goal()

        prev = None
        for i, node in enumerate(tqdm(self.path)):
            if len(node.hist_traj) > 0:
                cen_x = int(node.x /r)
                cen_y = y - int(node.y/r)
                if i % 2 == 0:
                    cv2.circle(self.map, (cen_x, cen_y), int(self.radius/r),
                            (0,0,255),2)
                traj = np.array(node.hist_traj)
                traj[:,0] /= r
                traj[:,1] = y - traj[:,1]/r
                cv2.polylines(self.map, [traj.astype(int)], False, (0,0,255),2)
                out.write(self.map.astype(np.uint8))
            else:
                cen_x = int(node.x /r)
                cen_y = y - int(node.y/r)
                cv2.circle(self.map, (cen_x, cen_y), int(self.radius/r),
                            (0,0,255),2)

        for i in range(10):
            out.write(self.map.astype(np.uint8))

        out.release()
        

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-s','--start', nargs='+', type=float, required=True)
    parser.add_argument('-g','--goal', nargs='+', type=float, required=True)
    parser.add_argument('-rpm','--RPM', nargs='+', type=str, required=True)
    parser.add_argument('-c','--clearance', type=float, default=0.1)
    parser.add_argument('-f','--file_name', type=str, default="visualize")
    args = parser.parse_args()

    if len(args.start) != 3:
        raise Exception("{} is an invalid start position. Should be size of 3".format(args.start))
    if len(args.goal) != 2:
        raise Exception("{} is an invalid goal position. Should be size of 2".format(args.goal))
    if len(args.RPM) != 4:  # ROS have 2 init arguments after the last arguments input
        raise Exception("{} is an invalid RPM input, Should be size of 2".format(args.RPM))

    args.start[0] = args.start[0] + 5
    args.start[1] = args.start[1] + 5
    args.start[2] = args.start[2] * 180 / np.pi

    args.goal[0] = args.goal[0] + 5
    args.goal[1] = args.goal[1] + 5
    RPM_tuple = (float(args.RPM[0]), float(args.RPM[1]))
    ROBOT_L = 0.354
    WHEEL_RADIUS = 0.038
    TIME_STEP = 1

    # Initialize Map
    # map size (x), map size (y), clearance, diameter of robot, radius of wheel,
    # rpm(left/right), goal threshold, resolution for searching ([x,y], theta)
    # resolution for visualize

    astar = Astar(10, 10,                 #map_size (x,y) 
                  args.clearance,         # clearance 
                  ROBOT_L,                # Robot diameter 
                  WHEEL_RADIUS,           # Wheel radius 
                  TIME_STEP,              # Time duration for each step
                  0.2,                    # goal threshold
                  RPM_tuple,             # rpm for left/rigth wheel
                  (0.1, 15),              # resolution for searching (position/radius)
                   0.01)                  # resolution for visualization

    # Specify the start position and end position
    # set_start_goal((X,Y,THETA), (X,Y))
    astar.set_start_goal(tuple(args.start), tuple(args.goal))

    astar.search()
    
    # Save to video
    # COMMENT OUT TO VISUALIZE
    astar.visualize("{}.mp4".format(args.file_name))


