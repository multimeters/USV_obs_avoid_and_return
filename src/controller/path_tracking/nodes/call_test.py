#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
# 导入上面的所有函数
from los_controller import controller  # 假设上面代码放在 your_controller_file.py

odom_data = None
path_data = None

def odom_callback(msg):
    global odom_data
    odom_data = msg

def path_callback(msg):
    global path_data
    path_data = msg

def main():
    rospy.init_node('controller_python_node', anonymous=True)

    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("/ow/local_path", Path, path_callback)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(50)  # 原代码循环频率 50 Hz

    while not rospy.is_shutdown():
        if odom_data is not None and path_data is not None:
            cmd_vel = controller(odom_data, path_data)
            cmd_pub.publish(cmd_vel)
        
        rate.sleep()

if __name__ == '__main__':
    main()
