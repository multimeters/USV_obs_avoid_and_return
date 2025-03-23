#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion

# 全局变量，用于模拟 C++ 静态变量
count = 0
ErrLastTime = 0.0
ErrRate = 0.0

def pid_control(LosAngle, theta):
    """
    LosAngle: 期望方向（角度，单位：度）
    theta:    当前朝向（角度，单位：度）
    返回:     PID 输出（角速度，尚未限幅）
    """
    global count, ErrLastTime, ErrRate

    # 计算误差（两者均已是 -180~180 的度数）
    Err = LosAngle - theta
    
    # 确保误差在 -180~180 之间
    if Err < -180:
        Err += 360
    if Err > 180:
        Err -= 360

    # 每隔一定步长再计算一次误差变化率（模拟采样）
    if count % 10 == 0:
        ErrRate = (Err - ErrLastTime) * 5  # 对应了原 C++ 中 (Err - ErrLastTime)*5
        ErrLastTime = Err
    
    count += 1
    
    # 这里对应原代码的控制参数（P=0.035，D=0.023）
    Angle = - (0.035 * Err + 0.023 * ErrRate)
    return Angle

def calc_theta(odom_msg):
    """
    从 odom 中提取 yaw(弧度) 并转换为度数。
    返回: yaw_degrees, 范围大约是 -180~180
    """
    orientation_q = odom_msg.pose.pose.orientation
    # 用 euler_from_quaternion 获取 (roll, pitch, yaw)，默认返回弧度
    (roll, pitch, yaw) = euler_from_quaternion([
        orientation_q.x, 
        orientation_q.y, 
        orientation_q.z, 
        orientation_q.w
    ])
    # 转换为度数
    yaw_degrees = yaw * 180.0 / math.pi
    return yaw_degrees

def change_angle_cal(x, y, path_msg):
    """
    计算从当前位置 (x, y) 到路径上“向前约 5 米的预测点” 的角度（度数）。
    path_msg: nav_msgs/Path
    返回: 预测方向角（单位：度）
    """
    path_length = len(path_msg.poses)
    if path_length < 2:
        # 若轨迹点过少，则直接返回 0
        return 0.0

    # 1. 找到离当前 (x,y) 距离最近的点的索引
    min_length = float('inf')
    min_index = 0
    
    for i in range(path_length - 5):
        xx = path_msg.poses[i].pose.position.x
        yy = path_msg.poses[i].pose.position.y
        dist = math.sqrt((x - xx)**2 + (y - yy)**2)
        if dist < min_length:
            min_length = dist
            min_index = i

    # 2. 从该最近点开始向后累加距离，直到约 5 米的位置
    sum_distance = 0.0
    predict_index = min_index  # 若找不到足够远，至少保证不越界
    for j in range(min_index, path_length - 1):
        x1 = path_msg.poses[j].pose.position.x
        y1 = path_msg.poses[j].pose.position.y
        x2 = path_msg.poses[j + 1].pose.position.x
        y2 = path_msg.poses[j + 1].pose.position.y
        
        segment_dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        sum_distance += segment_dist
        if sum_distance > 15.0:
            predict_index = j + 1
            break
    
    # 3. 计算该预测点与当前 (x,y) 的方向角
    predict_x = path_msg.poses[predict_index].pose.position.x
    predict_y = path_msg.poses[predict_index].pose.position.y

    dx = predict_x - x
    dy = predict_y - y
    angle_radians = math.atan2(dy, dx)
    angle_degrees = angle_radians * 180.0 / math.pi
    return angle_degrees

def controller(odom_msg, path_msg):
    """
    controller 函数：传入里程计 (nav_msgs/Odometry) 和规划路径 (nav_msgs/Path)，
    返回 geometry_msgs/Twist。
    
    - 若没有规划路径，则线速度=0,角速度=0。
    - 若有规划路径，则按照 C++ 中的逻辑进行 PID 运算并返回控制速度。
    """
    cmd_vel = Twist()
    path_length = len(path_msg.poses)

    # 若无有效路径，则停止
    if path_length == 0:
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        return cmd_vel
    
    # 1. 计算车辆当前位姿
    base_link_x = odom_msg.pose.pose.position.x
    base_link_y = odom_msg.pose.pose.position.y
    # 车辆朝向（度数）
    current_theta = calc_theta(odom_msg)
    
    # 2. 从路径中找到预测点的方向角
    pre_point_angle = change_angle_cal(base_link_x, base_link_y, path_msg)
    
    # 3. 用 PID 控制，计算需要的角速度(限幅前)
    angle_cmd = pid_control(pre_point_angle, current_theta)
    
    # 4. 线速度设置
    cmd_vel.linear.x = 1.0  # 这里根据需要修改，原 C++ 是 Move_flag=true 时线速度=1
    
    # 5. 角速度限幅
    if angle_cmd > 0.5:
        angle_cmd = 0.5
    elif angle_cmd < -0.5:
        angle_cmd = -0.5
    
    cmd_vel.angular.z = angle_cmd
    return cmd_vel
