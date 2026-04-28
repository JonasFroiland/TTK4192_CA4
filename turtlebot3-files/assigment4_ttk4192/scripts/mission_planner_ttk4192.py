#!/usr/bin/env python3
import rospy
import os
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2, tan
from os import system, name
import time
import re
import fileinput
import sys
import argparse
import random
import matplotlib.animation as animation
from datetime import datetime
from matplotlib.collections import PatchCollection, LineCollection
from matplotlib.patches import Rectangle
from itertools import product
from utils.astar import Astar
from utils.utils import plot_a_car, get_discretized_thetas, round_theta, same_point
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import shutil
import copy
import subprocess
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# Import here the packages used in your codes

""" ----------------------------------------------------------------------------------
Mission planner for Autonomos robots: TTK4192,NTNU. 
Date:20.03.23
characteristics: AI planning,GNC, hybrid A*, ROS.
robot: Turtlebot3
version: 1.1
""" 


# 1) Program here your AI planner 
"""
Graph plan ---------------------------------------------------------------------------
"""
class GraphPlan(object):
    def __init__(self, domain, problem):
        self.independentActions = []
        self.noGoods = []
        self.graph = []

    def graphPlan(self):
        # initialization
        initState = self.initialState

    def extract(self, Graph, subGoals, level):

        if level == 0:
            return []
        if subGoals in self.noGoods[level]:
            return None
        plan = self.gpSearch(Graph, subGoals, [], level)
        if plan is not None:
            return plan
        self.noGoods[level].append([subGoals])
        return None


#2) GNC module (path-followig and PID controller for the robot)
"""  Robot GNC module ----------------------------------------------------------------------
"""
class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        PI = 3.1415926535897
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class turtlebot_move():
    """
    Path-following module
    """
    def __init__(self):
        #rospy.init_node('turtlebot_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) # subscribing to the odometer
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)        # reading vehicle speed
        self.vel = Twist()
        self.rate = rospy.Rate(10)
        self.counter = 0
        self.trajectory = list()

        # track a sequence of waypoints
        #for point in WAYPOINTS[1:]:
        #    self.move_to_point(point[0], point[1])
        #    rospy.sleep(1)
        #self.stop()
        #rospy.logwarn("Action done.")


        # plot trajectory
        ####################################
        #data = np.array(self.trajectory)
        #np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        #plt.plot(data[:,0],data[:,1])
        #plt.show()
        ####################################

    def follow_waypoints(self, waypoints):
        for point in waypoints[1:]:
            self.move_to_point(point[0], point[1])
            rospy.sleep(1)

        self.stop()
        rospy.logwarn("Action done.")

    def move_to_point(self, x, y):
        # Here must be improved the path-following ---
        # Compute orientation for angular vel and direction vector for linear velocity

        diff_x = x - self.x
        diff_y = y - self.y
        #direction_vector = np.array([diff_x, diff_y])
        #direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        dist = sqrt(diff_x*diff_x + diff_y*diff_y)
        if dist < 0.05:
        	return
        direction_vector = np.array([diff_x, diff_y])/dist
        
        theta = atan2(diff_y, diff_x)

        # We should adopt different parameters for different kinds of movement
        self.pid_theta.setPID(1, 0, 0)     # P control while steering
        self.pid_theta.setPoint(theta)
        rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")

        
        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2
            if abs(angular) < 0.01:
                break
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.pid_theta.setPoint(theta)
        self.pid_theta.setPID(1, 0.02, 0.2)  # PID control while moving

        # Move to the target point
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) # projection
            if abs(linear) > 0.2:
                linear = linear/abs(linear)*0.2

            angular = self.pid_theta.update(self.theta)
            if abs(angular) > 0.2:
                angular = angular/abs(angular)*0.2

            if abs(linear) < 0.01 and abs(angular) < 0.01:
                break
            self.vel.linear.x = 1.5*linear   # Here can adjust speed
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()
        self.stop()
    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            #rospy.loginfo("odom: x=" + str(self.x) + ";  y=" + str(self.y) + ";  theta=" + str(self.theta))



# 3) Program here your path-finding algorithm
""" Hybrid A-star pathfinding --------------------------------------------------------------------
"""

# 3) Program here your path-finding algorithm
""" Hybrid A-star pathfinding --------------------------------------------------------------------
"""

class Node:
    """ Hybrid A* tree node. """

    def __init__(self, grid_pos, pos):
        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.g_ = None
        self.f = None
        self.parent = None
        self.phi = 0
        self.m = None
        self.branches = []

    def __eq__(self, other):
        return self.grid_pos == other.grid_pos

    def __hash__(self):
        return hash((self.grid_pos))


class HybridAstar:
    """ Hybrid A* search procedure. """

    def __init__(self, car, grid, reverse, unit_theta=pi/12, dt=1e-2, check_dubins=1):
        self.car = car
        self.grid = grid
        self.reverse = reverse
        self.unit_theta = unit_theta
        self.dt = dt
        self.check_dubins = check_dubins

        self.start = self.car.start_pos
        self.goal  = self.car.end_pos

        self.r = self.car.l / tan(self.car.max_phi)

        step_length = 0.5 * self.grid.cell_size
        self.drive_steps = int(step_length / self.dt) + 1
        self.arc = self.drive_steps * self.dt

        self.phil = [
            -self.car.max_phi,
            -2*self.car.max_phi/3,
            -self.car.max_phi/3,
             0,
             self.car.max_phi/3,
             2*self.car.max_phi/3,
             self.car.max_phi
        ]
        self.ml = [1, -1]

        if reverse:
            self.comb = list(product(self.ml, self.phil))
        else:
            self.comb = list(product([1], self.phil))

        from utils.dubins_path import DubinsPath
        self.dubins = DubinsPath(self.car)
        self.astar  = Astar(self.grid, self.goal[:2])

        self.w1 = 0.95
        self.w2 = 0.05
        self.w3 = 1.50  # penalty for changing steering angle (0.3)
        self.w4 = 0.80  # penalty for steering/turning (0.1)
        self.w5 = 3.00  # penalty for reverse (2.0)

        self.thetas = get_discretized_thetas(self.unit_theta)

    def construct_node(self, pos):
        theta    = round_theta(pos[2] % (2*pi), self.thetas)
        cell_id  = self.grid.to_cell_id(pos[:2])
        grid_pos = cell_id + [theta]
        return Node(grid_pos, pos)

    def simple_heuristic(self, pos):
        return abs(self.goal[0]-pos[0]) + abs(self.goal[1]-pos[1])

    def astar_heuristic(self, pos):
        # Fix 1 — handle None return from astar
        result = self.astar.search_path(pos[:2])
        if result is None:
            return self.simple_heuristic(pos[:2])
        h1 = result * self.grid.cell_size
        h2 = self.simple_heuristic(pos[:2])
        return self.w1*h1 + self.w2*h2

    def get_children(self, node, heu, extra):
        children = []
        for m, phi in self.comb:
            if node.m and node.phi == phi and node.m*m == -1:
                continue
            if node.m and node.m == 1 and m == -1:
                continue

            pos    = node.pos
            branch = [m, pos[:2]]
            for _ in range(self.drive_steps):
                pos = self.car.step(pos, phi, m)
                branch.append(pos[:2])

            pos1 = node.pos if m == 1 else pos
            pos2 = pos      if m == 1 else node.pos
            if phi == 0:
                safe = self.dubins.is_straight_route_safe(pos1, pos2)
            else:
                d, c, r = self.car.get_params(pos1, phi)
                safe    = self.dubins.is_turning_route_safe(pos1, pos2, d, c, r)

            if not safe:
                continue

            child         = self.construct_node(pos)
            child.phi     = phi
            child.m       = m
            child.parent  = node
            child.g       = node.g  + self.arc
            child.g_      = node.g_ + self.arc

            if extra:
                if phi != node.phi:
                    child.g += self.w3 * self.arc
                if phi != 0:
                    child.g += self.w4 * self.arc
                if m == -1:
                    child.g += self.w5 * self.arc

            child.f = child.g + (self.simple_heuristic(child.pos)
                                  if heu == 0 else
                                  self.astar_heuristic(child.pos))

            children.append([child, branch])
        return children

    def best_final_shot(self, open_, closed_, best, cost, d_route, n=10):
        open_.sort(key=lambda x: x.f)
        for t in range(min(n, len(open_))):
            best_       = open_[t]
            solutions_  = self.dubins.find_tangents(best_.pos, self.goal)
            d_route_, cost_, valid_ = self.dubins.best_tangent(solutions_)
            if valid_ and cost_ + best_.g_ < cost + best.g_:
                best, cost, d_route = best_, cost_, d_route_
        if best in open_:
            open_.remove(best)
            closed_.append(best)
        return best, cost, d_route

    def backtracking(self, node):
        route = []
        while node.parent:
            route.append((node.pos, node.phi, node.m))
            node = node.parent
        return list(reversed(route))

    def search_path(self, heu=1, extra=False):
        """ Hybrid A* pathfinding. """
        root    = self.construct_node(self.start)
        root.g  = float(0)
        root.g_ = float(0)
        root.f  = root.g + (self.simple_heuristic(root.pos)
                             if heu == 0 else
                             self.astar_heuristic(root.pos))

        closed_ = []
        open_   = [root]
        count   = 0

        while open_ and count < 5000:
            count += 1
            best  = min(open_, key=lambda x: x.f)
            open_.remove(best)
            closed_.append(best)

            if count % self.check_dubins == 0:
                solutions          = self.dubins.find_tangents(best.pos, self.goal)
                d_route, cost, valid = self.dubins.best_tangent(solutions)
                if valid:
                    best, cost, d_route = self.best_final_shot(
                        open_, closed_, best, cost, d_route)
                    route = self.backtracking(best) + d_route
                    path  = self.car.get_path(self.start, route)
                    cost += best.g_
                    print('Shortest path: {}'.format(round(cost, 2)))
                    print('Total iterations:', count)
                    return path, closed_

            for child, branch in self.get_children(best, heu, extra):
                if child in closed_:
                    continue
                if child not in open_:
                    best.branches.append(branch)
                    open_.append(child)
                elif child.g < open_[open_.index(child)].g:
                    best.branches.append(branch)
                    c = open_[open_.index(child)]
                    p = c.parent
                    for b in p.branches:
                        if same_point(b[-1], c.pos[:2]):
                            p.branches.remove(b)
                            break
                    open_.remove(child)
                    open_.append(child)
        print("Hybrid A* stopped: max iterations reached")

        return None, None


# Fix 2 — corrected obstacle map matching world file exactly
# Format: [x_bottom_left, y_bottom_left, width, height, safe_dis]
class map_grid_robplan:
    """
    Obstacle map matching the Gazebo world file.
    Positions converted from SDF center pose to bottom-left corner.
    Map dimensions: 5.21 x 2.75 m
    """
    def __init__(self):
	    self.obs = [
		# Outer walls
		[0.0,   0.0,   3.3,   0.05,  0.06],
		[3.275, 0.0,   0.05,  0.2,   0.06],
		[3.3,   0.175, 1.91,  0.05,  0.06],
		[0.0,   2.725, 5.21,  0.05,  0.06],
		[0.0,   0.0,   0.025, 2.75,  0.06],
		[5.185, 0.0,   0.025, 2.75,  0.06],

		# West wall protrusion
		[0.0,   1.0,   0.5,   0.2,   0.08],

		# Interior obstacles
		[1.2,   1.65,  0.2,   0.4,   0.08],  # obstacle_1
		[2.5,   1.65,  0.4,   0.4,   0.08],  # obstacle_2

		# Equipment boxes
		[1.55,  0.7,   0.5,   0.2,   0.02],  # equipment_valve0
		[3.06,  0.7,   0.5,   0.2,   0.02],  # equipment_valve1
		[3.61,  1.85,  0.4,   0.2,   0.02],  # equipment_pump1
	    ]


# Fix 5 — find_safe_pos helper
def find_safe_pos(env, car_l, x, y, heading, search_radius=0.5, steps=20):
    """
    Find a safe position near (x, y) by searching in expanding circles.
    Returns adjusted [x, y, heading] or None if not found.
    """
    from utils.car import SimpleCar
    dummy_car = SimpleCar(env, [x, y, heading], [x, y, heading], l=car_l)
    if dummy_car.is_pos_safe([x, y, heading]):
        return [x, y, heading]
    for r in np.linspace(0.05, search_radius, steps):
        for angle in np.linspace(0, 2*pi, 36):
            nx = x + r * np.cos(angle)
            ny = y + r * np.sin(angle)
            if dummy_car.is_pos_safe([nx, ny, heading]):
                print('  Adjusted ({:.2f},{:.2f}) -> ({:.2f},{:.2f})'.format(
                    x, y, nx, ny))
                return [nx, ny, heading]
    return None


def main_hybrid_a(heu, start_pos, end_pos, reverse, extra, grid_on):
    from utils.environment import Environment
    from utils.car import SimpleCar
    from utils.grid import Grid

    tc   = map_grid_robplan()
    env  = Environment(tc.obs, lx=5.21, ly=2.75)

    # Fix 3 — use l=0.3 and fix 5 — find safe positions
    start_pos = find_safe_pos(env, 0.3, start_pos[0], start_pos[1], start_pos[2])
    end_pos   = find_safe_pos(env, 0.3, end_pos[0],   end_pos[1],   end_pos[2])

    if start_pos is None or end_pos is None:
        print('Could not find safe start or end position.')
        return False

    car  = SimpleCar(env, start_pos, end_pos, l=0.3)  
    grid = Grid(env)

    hastar = HybridAstar(car, grid, reverse, unit_theta=pi/36)

    t = time.time()
    path, closed_ = hastar.search_path(heu, extra)
    print('Total time: {}s'.format(round(time.time()-t, 3)))

    if not path:
        print('No valid path!')
        return False

    path = path[::5] + [path[-1]]

    branches = []
    bcolors  = []
    for node in closed_:
        for b in node.branches:
            branches.append(b[1:])
            bcolors.append('y' if b[0] == 1 else 'b')

    xl, yl   = [], []
    xl_np1, yl_np1 = [], []
    carl = []
    dt_s = int(10)

    for i in range(len(path)):
        xl.append(path[i].pos[0])
        yl.append(path[i].pos[1])
        carl.append(path[i].model[0])
        if i == 0 or i == len(path)-1:
            xl_np1.append(path[i].pos[0])
            yl_np1.append(path[i].pos[1])
        elif dt_s*i < len(path):
            xl_np1.append(path[i*dt_s].pos[0])
            yl_np1.append(path[i*dt_s].pos[1])

    global WAYPOINTS
    WAYPOINTS = np.column_stack([np.array(xl_np1), np.array(yl_np1)])

    start_state = car.get_car_state(car.start_pos)
    end_state   = car.get_car_state(car.end_pos)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect("equal")

    if grid_on:
        ax.set_xticks(np.arange(0, env.lx, grid.cell_size))
        ax.set_yticks(np.arange(0, env.ly, grid.cell_size))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.tick_params(length=0)
        plt.grid(which='both')
    else:
        ax.set_xticks([])
        ax.set_yticks([])

    for ob in env.obs:
        ax.add_patch(Rectangle((ob.x, ob.y), ob.w, ob.h, fc='gray', ec='k'))

    ax.plot(car.start_pos[0], car.start_pos[1], 'ro', markersize=6)
    ax = plot_a_car(ax, end_state.model)
    ax = plot_a_car(ax, start_state.model)

    _branches = LineCollection([], linewidth=1)
    ax.add_collection(_branches)
    _path,  = ax.plot([], [], color='lime', linewidth=2)
    _carl   = PatchCollection([])
    ax.add_collection(_carl)
    _path1, = ax.plot([], [], color='w', linewidth=2)
    _car    = PatchCollection([])
    ax.add_collection(_car)

    frames = len(branches) + len(path) + 1

    def init():
        _branches.set_paths([])
        _path.set_data([], [])
        _carl.set_paths([])
        _path1.set_data([], [])
        _car.set_paths([])
        return _branches, _path, _carl, _path1, _car

    def animate(i):
        edgecolor = ['k']*5 + ['r']
        facecolor = ['y'] + ['k']*4 + ['r']
        if i < len(branches):
            _branches.set_paths(branches[:i+1])
            _branches.set_color(bcolors)
        else:
            _branches.set_paths(branches)
            j = i - len(branches)
            _path.set_data(xl[min(j, len(path)-1):], yl[min(j, len(path)-1):])
            sub_carl = carl[:min(j+1, len(path))]
            _carl.set_paths(sub_carl[::4])
            _carl.set_edgecolor('k')
            _carl.set_facecolor('m')
            _carl.set_alpha(0.1)
            _carl.set_zorder(3)
            _path1.set_data(xl[:min(j+1, len(path))], yl[:min(j+1, len(path))])
            _path1.set_zorder(3)
            _car.set_paths(path[min(j, len(path)-1)].model)
            _car.set_edgecolor(edgecolor)
            _car.set_facecolor(facecolor)
            _car.set_zorder(3)
        return _branches, _path, _carl, _path1, _car

    # ani = animation.FuncAnimation(fig, animate, init_func=init,
    #                               frames=frames, interval=1,
    #                               repeat=True, blit=True)
    # plt.show()
    plt.close(fig)

    return True
    

# Fix 4 and 6 — updated compute_path using real waypoint coordinates
def compute_path(wp_from, wp_to, heading_from=0, heading_to=0):
    """
    Compute and execute path between two waypoints.
    wp_from, wp_to: [x, y] coordinates from your waypoint table.
    """
    p = argparse.ArgumentParser()
    p.add_argument('-heu', type=int, default=0)
    p.add_argument('-r',   action='store_true')
    p.add_argument('-e',   action='store_true')
    p.add_argument('-g',   action='store_true')
    args = p.parse_args([])

    start_pos = [wp_from[0], wp_from[1], heading_from]
    end_pos   = [wp_to[0],   wp_to[1],   heading_to]
    #return main_hybrid_a(args.heu, start_pos, end_pos, args.r, args.e, args.g)
    return main_hybrid_a(args.heu, start_pos, end_pos, True, True, args.g)


# Waypoint coordinates — use these when calling compute_path
WP0 = [0.1,  0.2 ]
WP1 = [1.80, 0.35]
WP2 = [3.31, 1.3 ]
WP3 = [3.21, 2.70]
WP4 = [4.9,  0.4 ]
WP5 = [0.80, 2.00]
WP6 = [3.81, 1.5]



WAYPOINT_MAP = {
    "waypoint0": WP0,
    "waypoint1": WP1,
    "waypoint2": WP2,
    "waypoint3": WP3,
    "waypoint4": WP4,
    "waypoint5": WP5,
    "waypoint6": WP6,
}

WP_ANGLE = {
    "waypoint0": 0.0,
    "waypoint1": pi/2,      # face valve0
    "waypoint2": -pi/2,     # face valve1
    "waypoint3": 0.0,
    "waypoint4": 0.0,
    "waypoint5": pi/2,      # face pump0
    "waypoint6": pi/2       # face pump1
}


# STP Temporal Planner paths
STP_DIR     = os.path.expanduser("~") + "/Documents/teknarobotics/TTK4192_CA4/catkin_ws/src/temporal-planning-main/temporal-planning"
STP_DOMAIN  = STP_DIR + "/domains/ttk4192/domain/PDDL_domain_1.pddl"
STP_PROBLEM = STP_DIR + "/domains/ttk4192/problem/PDDL_problem_1.pddl"
STP_PLAN    = STP_DIR + "/tmp_sas_plan"

# Waypoints where pumps live (used to route take_picture to the right executor branch)
PUMP_WAYPOINTS = {"waypoint5", "waypoint6"}


def run_stp_planner():
    """
    Call the STP temporal planner as a subprocess and return a time-sorted
    list of raw action strings, e.g. ["move turtlebot0 waypoint0 waypoint1 d01", ...].
    Returns None if the planner fails or produces no output file.
    """
    if os.path.exists(STP_PLAN):
        os.remove(STP_PLAN)
    cmd = ["python2.7", "bin/plan.py", "stp-2", STP_DOMAIN, STP_PROBLEM]
    print("Running STP planner...")
    result = subprocess.run(cmd, cwd=STP_DIR, capture_output=True, text=True)
    print(result.stdout)
    if result.returncode != 0:
        print("STP planner failed:", result.stderr)
        return None

    if not os.path.exists(STP_PLAN):
        print("Plan file not found:", STP_PLAN)
        return None

    actions = []
    with open(STP_PLAN, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            match = re.search(r'\(\s*(.*?)\s*\)', line)
            if not match:
                continue
            time_match = re.match(r'([\d.]+):', line)
            start_time = float(time_match.group(1)) if time_match else 0.0
            actions.append((start_time, match.group(1)))

    actions.sort(key=lambda x: x[0])
    return [a[1] for a in actions]


def translate_stp_action(action_str):
    """
    Convert a raw STP action string to the token format expected by the
    executor's if/elif dispatch below.

    STP output          →  executor token
    ──────────────────────────────────────────────────────
    move ... FROM TO r  →  move_robot ... FROM TO r
    take_picture ... WP →  check_pump_picture_ir ...      (if WP is a pump waypoint)
                        →  check_seals_valve_picture_eo ... (otherwise)
    manipulate_valve ...→  manipulate_valve ...
    charge_battery ...  →  charge_battery ...
    charge ...          →  charge_battery ...
    """
    tokens = action_str.split()
    name = tokens[0]

    if name == "move":
        return "move_robot " + " ".join(tokens[1:])

    elif name == "take_picture":
        wp = tokens[2] if len(tokens) > 2 else ""
        if wp in PUMP_WAYPOINTS:
            return "check_pump_picture_ir " + " ".join(tokens[1:])
        else:
            return "check_seals_valve_picture_eo " + " ".join(tokens[1:])

    elif name == "manipulate_valve":
        return "manipulate_valve " + " ".join(tokens[1:])

    elif name in ("charge_battery", "charge"):
        return "charge_battery " + " ".join(tokens[1:])

    # unknown — pass through unchanged so it shows up as unrecognised in the log
    return action_str


#4) Program here the turtlebot actions (based in your AI planner)
"""
Turtlebot 3 actions-------------------------------------------------------------------------
"""
        
class TakePhoto:
    def __init__(self, topic_name="/camera/rgb/image_raw"):
        self.bridge = CvBridge()
        self.topic_name = topic_name

    def take_picture(self, img_path, timeout=5.0):
        try:
            msg = rospy.wait_for_message(self.topic_name, Image, timeout=timeout)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok = cv2.imwrite(img_path, cv_image)
            return ok
        except Exception as e:
            rospy.logerr("Failed to capture image from %s: %s", self.topic_name, str(e))
            return False
 

def take_inspection_image_EO(prefix, topic_name="/camera/rgb/image_raw"):
    camera = TakePhoto(topic_name)

    now = datetime.now()
    dt_string = now.strftime("%d%m%Y_%H%M%S")
    filename = prefix + dt_string + ".jpg"

    home_dir = os.path.expanduser("~")
    file_destination = home_dir + "/Documents/teknarobotics/TTK4192_CA4/turtlebot3-files/assigment4_ttk4192/scripts"
    img_path = os.path.join(file_destination, filename)

    os.makedirs(file_destination, exist_ok=True)

    if camera.take_picture(img_path, timeout=5.0):
        rospy.loginfo("Saved image " + img_path)
    else:
        rospy.loginfo("No image captured, photo not saved")

    rospy.sleep(1)    


# Move the robot
#def move_robot_waypoint0_waypoint1():
#    print("Computing Hybrid A* path WP0 -> WP1")
#    dx = WP1[0] - WP0[0]
#    dy = WP1[1] - WP0[1]
#    heading = float(np.arctan2(dy, dx))
#    compute_path(WP0, WP1, heading_from=heading, heading_to=heading)
#    print("Executing path following")
#    turtlebot_move()
def move_robot_between_waypoints(from_wp_name, to_wp_name, controller):
    if from_wp_name not in WAYPOINT_MAP or to_wp_name not in WAYPOINT_MAP:
        print(f"Unknown waypoint(s): {from_wp_name}, {to_wp_name}")
        time.sleep(1)
        return

    wp_from = WAYPOINT_MAP[from_wp_name]
    wp_to = WAYPOINT_MAP[to_wp_name]

    print(f"Computing Hybrid A* path {from_wp_name} -> {to_wp_name}")
    
    robot_x, robot_y, robot_theta = get_current_pose()

    dx = wp_to[0] - wp_from[0]
    dy = wp_to[1] - wp_from[1]
    goal_heading = float(np.arctan2(dy, dx))
    
    success = compute_path(
	    wp_from,
	    wp_to,
	    heading_from=robot_theta,
	    heading_to=goal_heading
	)

    
    if not success:
        print("Skipping movement because Hybrid A* failed.")
        return False


    print(f"Executing path following {from_wp_name} -> {to_wp_name}")
    controller.follow_waypoints(WAYPOINTS)
    return True
    

def making_turn_exe():
    print("Executing Make a turn")
    time.sleep(1)
    #Starts a new node
    #rospy.init_node('turtlebot_move', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    print("Let's rotate your robot")
    #speed = input("Input your speed (degrees/sec):")
    #angle = input("Type your distance (degrees):")
    #clockwise = input("Clockwise?: ") #True or false

    speed = 5
    angle = 180
    clockwise = True

    #Converting from angles to radians
    angular_speed = speed*2*pi/360
    relative_angle = angle*2*pi/360

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0   #should be from the odometer

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    #rospy.spin()


# Rotate to task
def get_closest_waypoint(x, y):
    waypoints = {
        "waypoint0": WP0,
        "waypoint1": WP1,
        "waypoint2": WP2,
        "waypoint3": WP3,
        "waypoint4": WP4,
        "waypoint5": WP5,
        "waypoint6": WP6
    }

    closest_wp = None
    min_dist = float('inf')

    for name, wp in waypoints.items():
        dx = wp[0] - x
        dy = wp[1] - y
        dist = sqrt(dx*dx + dy*dy)

        if dist < min_dist:
            min_dist = dist
            closest_wp = name

    return closest_wp
    

def get_current_pose():
    odom_msg = rospy.wait_for_message("odom", Odometry)
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y

    quat = [
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
    ]
    (_, _, theta) = tf.transformations.euler_from_quaternion(quat)

    return x, y, theta
    

def rotate_to_heading(target_theta):
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel = Twist()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        odom_msg = rospy.wait_for_message("odom", Odometry)

        quat = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        ]
        (_, _, current_theta) = tf.transformations.euler_from_quaternion(quat)

        error = target_theta - current_theta
        if error > pi:
            error -= 2*pi
        elif error < -pi:
            error += 2*pi

        if abs(error) < 0.05:
            break

        vel.linear.x = 0.0
        vel.angular.z = max(min(1.5 * error, 0.3), -0.3)
        vel_pub.publish(vel)
        rate.sleep()

    vel.linear.x = 0.0
    vel.angular.z = 0.0
    vel_pub.publish(vel)
    rospy.sleep(1)


# Take EO and IR pictures
def taking_photo_exe():
    take_inspection_image_EO("photo_")


def check_pump_picture_ir():
    print("Taking IR picture using EO camera ...")
    take_inspection_image_EO("pump_ir_")


def check_seals_valve_picture_eo():
    print("Taking EO picture ...")
    take_inspection_image_EO("valve_eo_")


# Charging battery 
def charge_battery():
    print("Charging battery ...")
    time.sleep(5)
    print("Battery charged")


# Arm joint positions (radians) for the OpenManipulator-X
_ARM_HOME     = [0.0, -1.05,  0.35,  0.70]   # safe upright rest pose
_ARM_REACH    = [0.0, -0.50,  0.30,  0.20]   # slight tilt down, stays close to robot body
_GRIPPER_OPEN  =  0.010                        # gripper fully open
_GRIPPER_CLOSE = -0.010                        # gripper closed (grasping)


def _send_arm_goal(positions, move_time=2.0):
    """Send a single joint-space trajectory point to the arm controller and wait."""
    client = actionlib.SimpleActionClient(
        '/arm_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    if not client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logwarn("arm_controller action server not available — skipping arm move")
        return False

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(move_time)

    traj = JointTrajectory()
    traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
    traj.points = [point]

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(move_time + 3.0))
    return True


def _send_gripper_goal(position, move_time=1.0):
    """Send a gripper open/close command and wait."""
    client = actionlib.SimpleActionClient(
        '/gripper_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction
    )
    if not client.wait_for_server(timeout=rospy.Duration(5.0)):
        rospy.logwarn("gripper_controller action server not available — skipping gripper move")
        return False

    point = JointTrajectoryPoint()
    point.positions = [position]
    point.time_from_start = rospy.Duration(move_time)

    traj = JointTrajectory()
    traj.joint_names = ['gripper']
    traj.points = [point]

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration(move_time + 2.0))
    return True


def Manipulate_OpenManipulator_x():
    """
    Demonstrate arm and gripper movement without reaching far forward.
    Tilts the arm slightly down, opens/closes the gripper, then returns home.
    """
    print("Executing manipulate the robot arm to open valve")

    _send_gripper_goal(_GRIPPER_OPEN,  move_time=1.0)   # open gripper
    rospy.sleep(0.5)

    _send_arm_goal(_ARM_REACH, move_time=2.5)            # small tilt toward valve
    rospy.sleep(1.0)

    _send_gripper_goal(_GRIPPER_CLOSE, move_time=1.0)   # close gripper
    rospy.sleep(1.0)

    _send_gripper_goal(_GRIPPER_OPEN,  move_time=1.0)   # open gripper again
    rospy.sleep(0.5)

    _send_arm_goal(_ARM_HOME, move_time=2.5)             # return to home
    print("Manipulation complete")



# Define the global varible: WAYPOINTS  Wpts=[[x_i,y_i]];
global WAYPOINTS
#WAYPOINTS = [[1,1],[2,2]]
WAYPOINTS = np.array([WP0, WP1])

# 5) Program here the main commands of your mission planner code
""" Main code ---------------------------------------------------------------------------
"""
if __name__ == '__main__':
    try:
        rospy.init_node('mission_planner_ttk4192', anonymous=False)
        controller = turtlebot_move()
        print()
        print("************ TTK4192 - Assigment 4 **************************")
        print()
        print("AI planners: GraphPlan")
        print("Path-finding: Hybrid A-star")
        print("GNC Controller: PID path-following")
        print("Robot: Turtlebot3 waffle-pi")
        print("date: 20.03.23")
        print()
        print("**************************************************************")
        print()
        print("Press Intro to start ...")
        input_t=input("")
        # 5.0) Testing the GNC module (uncomment lines to test)

        # aea
        # aea 2
        # aea 3
        # aea 4
        
        # 5.1) Starting the AI Planner
        print(" --- Running STP Temporal Planner --- ")
        raw_actions = run_stp_planner()

        if raw_actions:
            plan_general = [translate_stp_action(a) for a in raw_actions]
            print("Plan from STP planner:")
            for i, step in enumerate(plan_general):
                print("  {}: {}".format(i+1, step))
        else:
            print("STP planner failed — using fallback plan")
            plan_general = [
                "move_robot turtlebot0 waypoint0 waypoint1 d01",
                "check_seals_valve_picture_eo turtlebot0 waypoint1",
                "move_robot turtlebot0 waypoint1 waypoint2 d12",
                "check_seals_valve_picture_eo turtlebot0 waypoint2",
                "move_robot turtlebot0 waypoint2 waypoint5 d25",
                "check_pump_picture_ir turtlebot0 waypoint5",
                "move_robot turtlebot0 waypoint5 waypoint6 d56",
                "check_pump_picture_ir turtlebot0 waypoint6",
                "move_robot turtlebot0 waypoint6 waypoint3 d63",
            ]
            
    
        # 5.2) Reading the plan 
        print("  ")
        print("Reading the plan from AI planner")
        print("  ")
        plan_general=plan_general
        print(plan_general[0])

        # 5.3) Start mission execution 
        # convert string into functions and executing
        print("")
        print("Starting mission execution")
        # Start simulations with battery = 100%
        battery=100
        task_finished=0
        task_total=len(plan_general)
        
        i_ini=0
        while i_ini < task_total:
            plan_temp = plan_general[i_ini].split()
            print(plan_temp)

            if plan_temp[0] == "move_robot":
                print("Executing move_robot action")
                if len(plan_temp) >= 4:
                    from_wp = plan_temp[2]
                    to_wp = plan_temp[3]
                    move_robot_between_waypoints(from_wp, to_wp, controller)
                else:
                    print("Invalid move_robot format")
                    time.sleep(1)

            elif plan_temp[0] == "taking_photo":
                print("Executing taking_photo action")

                robot_x, robot_y, _ = get_current_pose()
                closest_wp = get_closest_waypoint(robot_x, robot_y)
                print("Closest waypoint:", closest_wp)

                target_theta = WP_ANGLE[closest_wp]
                rotate_to_heading(target_theta)

                taking_photo_exe()

            elif plan_temp[0] == "check_pump_picture_ir":
                print("Executing check_pump_picture_ir action")

                robot_x, robot_y, _ = get_current_pose()
                closest_wp = get_closest_waypoint(robot_x, robot_y)
                print("Closest waypoint:", closest_wp)

                target_theta = WP_ANGLE[closest_wp]
                rotate_to_heading(target_theta)

                check_pump_picture_ir()

            elif plan_temp[0] == "check_seals_valve_picture_eo":
                print("Executing check_seals_valve_picture_eo action")

                robot_x, robot_y, _ = get_current_pose()
                closest_wp = get_closest_waypoint(robot_x, robot_y)
                print("Closest waypoint:", closest_wp)

                target_theta = WP_ANGLE[closest_wp]
                rotate_to_heading(target_theta)

                check_seals_valve_picture_eo()
            
            elif plan_temp[0] == "manipulate_valve":
                print("Executing manipulate_valve action not implemented ...")
                #Manipulate_OpenManipulator_x()
                
                print("Executing check_seals_valve_picture_eo action")

                robot_x, robot_y, _ = get_current_pose()
                closest_wp = get_closest_waypoint(robot_x, robot_y)
                print("Closest waypoint:", closest_wp)

                target_theta = WP_ANGLE[closest_wp]
                rotate_to_heading(target_theta)

                check_seals_valve_picture_eo()
                


            elif plan_temp[0] == "charge_battery":
                print("Executing charge action")

                robot_x, robot_y, _ = get_current_pose()
                closest_wp = get_closest_waypoint(robot_x, robot_y)
                print("Closest waypoint:", closest_wp)

                target_theta = WP_ANGLE[closest_wp]
                rotate_to_heading(target_theta)

                charge_battery()

            i_ini = i_ini + 1


        print("")
        print("--------------------------------------")
        print("All tasks were performed successfully")
        time.sleep(10)  

    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")