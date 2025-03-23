#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf.transformations import euler_from_quaternion

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    rospy.logerr("无法导入 ompl Python 接口，请确保已经安装了 OMPL 并且包含 Python bindings")
    exit()


class RRTStarPlanner(object):
    def __init__(self):
        # 订阅
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.map_sub = rospy.Subscriber('/obs_map', OccupancyGrid, self.map_callback)

        self.path_pub = rospy.Publisher('/ow/local_path', Path, queue_size=1)

        self.current_pose = None
        self.goal_pose = None
        self.map_data = None
        self.map_info = None

        self.rate = rospy.Rate(1)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.current_pose = (x, y, yaw)

    def goal_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])
        self.goal_pose = (x, y, yaw)
        rospy.loginfo("收到新的目标点: (%.2f, %.2f, %.2f)" % (x, y, yaw))

        if self.map_data is not None and self.current_pose is not None:
            self.plan_path()

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8)
        self.map_info = msg.info
        rospy.loginfo("已接收到地图数据。")

    def is_state_valid(self, state):
        x, y, _ = state
        if self.map_info is None:
            return False

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)

        if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
            return False

        idx = my * self.map_info.width + mx
        cell_value = self.map_data[idx]
        if cell_value == 100 or cell_value == -1:
            return False

        return True

    def plan_path(self):
        se2_space = ob.SE2StateSpace()
        # 设置空间边界
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        width_m = self.map_info.width * self.map_info.resolution
        height_m = self.map_info.height * self.map_info.resolution

        bounds = ob.RealVectorBounds(2)
        bounds.low[0] = origin_x
        bounds.low[1] = origin_y
        bounds.high[0] = origin_x + width_m
        bounds.high[1] = origin_y + height_m
        se2_space.setBounds(bounds)

        ss = og.SimpleSetup(se2_space)

        # 设置碰撞检测
        def validity_checker_fn(state):
            return self.is_state_valid([
                state.getX(),
                state.getY(),
                state.getYaw()
            ])
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(validity_checker_fn))

        si = ss.getSpaceInformation()

        # 替换掉 setLongestValidSegmentLength(0.05)，
        # 使用 setStateValidityCheckingResolution(0.005) 提升插值精度
        si.setStateValidityCheckingResolution(0.005)

        # 如果想进一步细化，可用更小值，比如 0.002 (但计算量会更多)
        # si.setStateValidityCheckingResolution(0.002)

        # 设定起点和终点
        start = ob.State(se2_space)
        goal = ob.State(se2_space)

        sx, sy, syaw = self.current_pose
        gx, gy, gyaw = self.goal_pose

        start().setX(sx)
        start().setY(sy)
        start().setYaw(syaw)

        goal().setX(gx)
        goal().setY(gy)
        goal().setYaw(gyaw)

        ss.setStartAndGoalStates(start, goal)

        planner = og.RRTstar(si)
        ss.setPlanner(planner)

        time_limit = 3.0
        solved = ss.solve(time_limit)

        if solved:
            rospy.loginfo("RRT* 规划成功，简化路径...")
            ss.simplifySolution()
            path_geometric = ss.getSolutionPath()
            self.publish_path(path_geometric)
        else:
            rospy.logwarn("RRT* 规划失败，未找到可行路径")

    def publish_path(self, path_geometric):
        raw_path = []
        for i in range(path_geometric.getStateCount()):
            state = path_geometric.getState(i)
            px = state.getX()
            py = state.getY()
            yaw = state.getYaw()
            raw_path.append((px, py, yaw))

        sampled_path = self.sample_path(raw_path, step=1.0)

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for (px, py, yaw) in sampled_path:
            pose = Pose()
            pose.position.x = px
            pose.position.y = py
            quat = self.yaw_to_quaternion(yaw)
            pose.orientation = Quaternion(*quat)

            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose = pose
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        rospy.loginfo("已发布采样后路径，采样点数：%d" % len(path_msg.poses))

    def sample_path(self, path, step=1.0):
        if len(path) < 2:
            return path

        new_path = [path[0]]
        for i in range(len(path) - 1):
            x_i, y_i, yaw_i = path[i]
            x_j, y_j, yaw_j = path[i+1]

            dx = x_j - x_i
            dy = y_j - y_i
            segment_len = math.sqrt(dx**2 + dy**2)

            if segment_len < step:
                new_path.append(path[i+1])
            else:
                yaw_diff = self.angle_diff(yaw_j, yaw_i)
                current_dist = 0.0
                while current_dist + step < segment_len:
                    current_dist += step
                    ratio = current_dist / segment_len
                    x_n = x_i + dx * ratio
                    y_n = y_i + dy * ratio
                    yaw_n = yaw_i + yaw_diff * ratio
                    yaw_n = self.normalize_angle(yaw_n)
                    new_path.append((x_n, y_n, yaw_n))

                new_path.append(path[i+1])

        return new_path

    @staticmethod
    def angle_diff(a, b):
        d = a - b
        d = (d + math.pi) % (2 * math.pi) - math.pi
        return d

    @staticmethod
    def normalize_angle(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def yaw_to_quaternion(self, yaw):
        half_yaw = yaw / 2.0
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)
        return (0.0, 0.0, sy, cy)

    def spin(self):
        rospy.loginfo("RRT* planner (old OMPL, no setLongestValidSegmentLength) node started.")
        while not rospy.is_shutdown():
            self.rate.sleep()


def main():
    rospy.init_node('rrt_star_planner_node', anonymous=True)
    planner = RRTStarPlanner()
    planner.spin()


if __name__ == '__main__':
    main()
