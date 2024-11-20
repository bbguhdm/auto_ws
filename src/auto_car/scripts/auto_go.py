#!/usr/bin/env python
# -*- coding: utf-8 -*-
import heapq
from math import pi, sqrt, atan2, cos, sin
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
# 定义线速度缩放因子函数
def scale_factor(angular_velocity):
    # 根据角速度定义一个平滑缩放因子函数
    if abs(angular_velocity) < 0.9:
        return 1  # 角速度较小时，不限制线速度
    elif abs(angular_velocity) < 1.9:
        return max(0.32, 1 - 0.8 * (abs(angular_velocity) - 0.9))  # 平滑缩减
    else:
        return 0.32  # 最大角速度限制
def get_path_from_A_star(start, goal, obstacles):
    def heuristic(a, b):
        # Using Euclidean distance as the heuristic
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def get_neighbors(node):
        # 4-connected neighbors (no diagonal movement)
        directions = [(0.25, 0), (-0.25, 0), (0.5, 0), (-0.5, 0),(1.0,0),(-1.0,0), (0, 0.25), (0, -0.25), (0, 0.5), (0, -0.5),(0,1),(0,-1)]
        neighbors = [(node[0] + d[0], node[1] + d[1]) for d in directions]
        return neighbors

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    obstacle_set = set(obstacles)

    while open_set:
        current_f, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for neighbor in get_neighbors(current):
            if neighbor in obstacle_set:
                continue

            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return []

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.prev_linear_velocity = 0
        self.prev_angular_velocity = 0
        self.prev_error_distance = 0
        self.prev_error_angle = 0
        self.integral_distance = 0
        self.integral_angle = 0
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def add_inflation_layer(self, obstacles, inflation_radius):
        inflated_obstacles = set(obstacles)
        for (x, y) in obstacles:
            for dx in np.arange(-inflation_radius, inflation_radius + 0.1, 0.25):  # 0.1是一个小增量
                for dy in np.arange(-inflation_radius, inflation_radius + 0.1, 0.25):
                    if dx ** 2 + dy ** 2 <= inflation_radius ** 2:
                        inflated_x = x + dx # 将浮点坐标转换为栅格坐标
                        inflated_y = y + dy
                        inflated_obstacles.add((inflated_x, inflated_y))
        return list(inflated_obstacles)
    def run(self):
        start = (0, 0)
        goal = (3.75, 1.25)

        obstacles = [
            (-0.5, -0.5), (0, -0.5), (0.5, -0.5), (1, -0.5), (1.5, -0.5), (2, -0.5), (2.5, -0.5), (3, -0.5),
            (-0.5, -0.75), (0, -0.75), (0.5, -0.75), (1, -0.75), (1.5, -0.75), (2, -0.75), (2.5, -0.75), (3, -0.75),
            (-0.5, 2), (0, 2), (0.5, 2), (1, 2), (1.5, 2), (2, 2), (2.5, 2), (3, 2), (3.5, 2), (4, 2), (4.5, 2),
            (-0.5, 2.25), (0, 2.25), (0.5, 2.25), (1, 2.25), (1.5, 2.25), (2, 2.25), (2.5, 2.25), (3, 2.25), (3.5, 2.25), (4, 2.25), (4.5, 2.25),
            (-0.5, 0), (-0.5, 0.5), (-0.5, 1), (-0.5, 1.5),
            (1, -0.25), (1, 0), (1, 0.25), (1, 0.5), (1, 0.75), (1, 1.0),
            (1.25, -0.25), (1.25, 0), (1.25, 0.25), (1.25, 0.5), (1.25, 0.75), (1.25, 1),
            (1.5, -0.25), (1.5, 0), (1.5, 0.25), (1.5, 0.5), (1.5, 0.75), (1.5, 1),
            (2.5, 0.5), (2.5, 0.75), (2.5, 1), (2.5, 1.25), (2.5, 1.5), (2.5, 1.75),
            (2.75, 0.5), (2.75, 0.75), (2.75, 1), (2.75, 1.25), (2.75, 1.5), (2.75, 1.75),
            (3, 0.5), (3, 0.75), (3, 1), (3, 1.25), (3, 1.5), (3, 1.75)
        ]
        obstacles=self.add_inflation_layer(obstacles,0.25)
        waypoints = get_path_from_A_star(start, goal, obstacles)
	waypoints.append((4.25,1.25))
        print(waypoints)
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1],len(waypoints)-1,i)

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        A = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [1, T, T ** 2, T ** 3],
                      [0, 1, 2 * T, 3 * T ** 2]])
        b = np.array([p_start, v_start, p_end, v_end])
        coeffs = np.linalg.solve(A, b)
        return coeffs

    def move_to_point(self, current_waypoint, next_waypoint, len, a, T=3.8):
        p_start_x, p_start_y = current_waypoint
        p_end_x, p_end_y = next_waypoint
        dt = 0.2
        t = 0

        # 设定最大速度
        max_linear_velocity = 0.9
        max_angular_velocity = 2.2

        # 设定 PID 控制器的参数
        Kp_linear = 3.2
        Ki_linear = 0.04  # 积分增益
        Kd_linear = 0.47   # 微分增益

        Kp_angular = 3.0
        Ki_angular = 0.02  # 积分增益
        Kd_angular = 0.31  # 微分增益

        while t <= T:
            x_t = p_start_x + (p_end_x - p_start_x) * (t / T)
            y_t = p_start_y + (p_end_y - p_start_y) * (t / T)
            distance_to_target = sqrt((x_t - self.pose.x) ** 2 + (y_t - self.pose.y) ** 2)
            if(distance_to_target < 0.03 and a != len):
                break

            target_theta = atan2(y_t - self.pose.y, x_t - self.pose.x)
            angle_diff = target_theta - self.pose.theta
            angle_diff = (angle_diff + pi) % (2 * pi) - pi

            # 线性 PID 控制
            error_distance = distance_to_target
            self.integral_distance += error_distance * dt
            derivative_distance = (error_distance - self.prev_error_distance) / dt

            linear_velocity = Kp_linear * error_distance + Ki_linear * self.integral_distance + Kd_linear * derivative_distance

            # 角度 PID 控制
            error_angle = angle_diff
            self.integral_angle += error_angle * dt
            derivative_angle = (error_angle - self.prev_error_angle) / dt

            angular_velocity = Kp_angular * error_angle + Ki_angular * self.integral_angle + Kd_angular * derivative_angle
            linear_velocity = max(min(linear_velocity, max_linear_velocity), -max_linear_velocity)
            angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)
            # 限制速度
            linear_velocity = linear_velocity * scale_factor(angular_velocity)
            # 发送速度指令
            vel_msg = Twist()
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity
            self.vel_pub.publish(vel_msg)

            # 更新前一个速度
            self.prev_linear_velocity = linear_velocity
            self.prev_angular_velocity = angular_velocity
            self.prev_error_distance = error_distance
            self.prev_error_angle = error_angle

            t += dt
            self.rate.sleep()
        vel_msg = Twist()
        self.vel_pub.publish(vel_msg)
    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])
            rospy.loginfo("odom: x=" + str(self.pose.x) + ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

if __name__ == '__main__':
    whatever = Turtlebot()
