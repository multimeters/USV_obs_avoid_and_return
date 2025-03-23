#!/usr/bin/env python3
import sys
sys.path.append('/home/lhl/baoliang/src/controller/path_tracking/nodes')
import rospy
from los_controller import controller 
from my_utils import myData
from my_utils import task_manager
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import time 

if __name__ == '__main__':
    rospy.init_node('path_angle_filter')
    tm= task_manager()
    #rospy.spin()
    rate = rospy.Rate(20)  # 20 Hz


    while not rospy.is_shutdown():
        #print(tm.step)
        #print(filter_node.step)
        try:
            if tm.step==1:
                if tm.is_path_colliding():
                    
                    #print("colliding")
                    twist_msg = Twist()
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = 0
                    # Publish the Twist message
                    tm.cmd_vel_pub.publish(twist_msg)
                    
                    tm.step=20
                else:
                    #print("no colliding")
                    if tm.compute_distance_to_goal(tm.searched_path)>15 :
                        print(tm.compute_distance_to_goal(tm.searched_path))
                        cmd_vel = controller(tm.odom_data, tm.searched_path)
                        #print(cmd_vel)
                        tm.cmd_vel_pub.publish(cmd_vel)
                    else:
                        twist_msg = Twist()
                        twist_msg.linear.x = 0
                        twist_msg.angular.z = 0
                        # Publish the Twist message
                        tm.cmd_vel_pub.publish(twist_msg)
                        tm.step=22
                # try:
                #     if(len(tm.path_dir)!=0):
                #         i=0
                #         while i < len(tm.path_dir):
                #             if tm.is_path_colliding() and tm.step==2:
                #                 rospy.logerr("Path is colliding with an obstacle.")
                #                 tm.step=3
                #                 #print(filter_node.step)
                #                 break
                #             print(tm.compute_distance_to_goal(tm.path_dir[i]))
                #             if tm.compute_distance_to_goal(tm.path_dir[i])>0.2:
                #                 if tm.path_is_forward[i]:
                #                     cmd_vel=PurePursuit.get_cmd_vel( tm.odom_data ,tm.path_dir[i],1)
                #                 else:
                #                     cmd_vel=PurePursuit.get_cmd_vel( tm.odom_data ,tm.path_dir[i],-1)
                #                 print(cmd_vel)
                #                 print(tm.compute_distance_to_goal(tm.path_dir[i]))
                #                 twist_msg = Twist()
                #                 twist_msg.linear.x = cmd_vel["linear"]
                #                 twist_msg.angular.z = cmd_vel["angular"]
                #                 # Publish the Twist message
                #                 tm.cmd_vel_pub.publish(twist_msg)
                #             else:
                #                 i=i+1
                #                 print(cmd_vel)
                #                 print(tm.compute_distance_to_goal(tm.path_dir[0]))
                #                 twist_msg = Twist()
                #                 twist_msg.linear.x = 0
                #                 twist_msg.angular.z = 0
                #                 # Publish the Twist message
                #                 tm.cmd_vel_pub.publish(twist_msg)
                #         if tm.step==2 and len(tm.waypoints)==0:   
                #             tm.step=0
                #         elif tm.step==2 and len(tm.waypoints)!=0:
                #             tm.step=5
                #     else:
                #         twist_msg = Twist()
                #         twist_msg.linear.x = 0
                #         twist_msg.angular.z = 0
                #         # Publish the Twist message
                #         tm.cmd_vel_pub.publish(twist_msg)
                # except :
                #     pass
            # elif filter_node.step==0:    
            #     twist_msg = Twist()
            #     twist_msg.linear.x = 0
            #     twist_msg.angular.z = 0
            #     # Publish the Twist message
            #     filter_node.cmd_vel_pub.publish(twist_msg)
            elif tm.step==20:
                tm.goal_pose_pub.publish(tm.searched_path.poses[-1])
                tm.searched_path=None
                tm.step=21
            elif tm.step==3:    
                #print(filter_node.step)
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                # Publish the Twist message
                tm.cmd_vel_pub.publish(twist_msg)
                time.sleep(4)
                # Check if the path contains at least two poses
                if len(tm.searched_path.poses) < 2:
                    rospy.logwarn('Received path with less than two poses. Ignoring...')
                    #break        

                tm.sub_map_pub.publish(tm.map_data)

                start_pose_msg = PoseWithCovarianceStamped()
                start_pose_msg.header.stamp = rospy.Time.now()
                start_pose_msg.header.frame_id = "map"  # or "odom" depending on your setup
                start_pose_msg.pose.pose.position.x = tm.odom_data.pose.pose.position.x
                start_pose_msg.pose.pose.position.y = tm.odom_data.pose.pose.position.y
                start_pose_msg.pose.pose.orientation = tm.odom_data.pose.pose.orientation
                tm.start_pose_pub.publish(start_pose_msg)

                goal_pose = tm.searched_path.poses[-1]
                print(goal_pose)
                goal_pose_stamped = PoseStamped()
                goal_pose_stamped = goal_pose
                tm.goal_pose_pub.publish(goal_pose_stamped)
                tm.step=1
            elif tm.step==5:  
                #print(filter_node.step)
                if(len(tm.waypoints)>=2 and tm.waypoints_cnt<len(tm.waypoints)):
                    twist_msg = Twist()
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = 0
                    # Publish the Twist message
                    tm.cmd_vel_pub.publish(twist_msg)
                    time.sleep(4)  

                    tm.sub_map_pub.publish(tm.map_data)

                    start_pose_msg = PoseWithCovarianceStamped()
                    start_pose_msg.header.stamp = rospy.Time.now()
                    start_pose_msg.header.frame_id = "map"  # or "odom" depending on your setup
                    start_pose_msg.pose.pose.position.x = tm.odom_data.pose.pose.position.x
                    start_pose_msg.pose.pose.position.y = tm.odom_data.pose.pose.position.y
                    start_pose_msg.pose.pose.orientation = tm.odom_data.pose.pose.orientation
                    tm.start_pose_pub.publish(start_pose_msg)

                    goal_pose_stamped = PoseStamped()
                    goal_pose_stamped = tm.waypoints[tm.waypoints_cnt]
                    tm.waypoints_cnt=tm.waypoints_cnt+1
                    tm.goal_pose_pub.publish(goal_pose_stamped)
                    tm.step=1
        except :
            pass
    rate.sleep()  # 保持循环的频率为10Hz

