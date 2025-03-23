#!/usr/bin/env python3
import rospy
import numpy as np
import threading
import time

from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.point_cloud2 import read_points, create_cloud
from tf.transformations import euler_from_quaternion
import std_msgs.msg
import message_filters

from std_srvs.srv import Empty, EmptyResponse
def delayed_start():
    # 等待参数中的延迟时间
    delay_time = rospy.get_param('start_delay', 10)  # 默认为 5 秒
    rospy.loginfo(f"Delaying node start for {delay_time} seconds...")
    time.sleep(delay_time)  # 延迟启动

    # 启动节点
    rospy.loginfo("Starting perception node...")
    rospy.launch_node("perception", "point_prejection.py")  # 启动 perception 节点

class MapProjection:
    def __init__(self):
        rospy.init_node('map_projection', anonymous=True)

        # ============== 获取静态地图 ==================
        self.map_client = rospy.ServiceProxy('/static_map', GetMap)
        self.obs_map = OccupancyGrid()
        self.map_data = None
        try:
            self.map_data = self.map_client().map
            self.obs_map.data  = np.array(self.map_data.data, dtype=np.int8)
            self.obs_map.header = self.map_data.header
            self.obs_map.info   = self.map_data.info
        except rospy.ServiceException as e:
            rospy.logerr("Failed to get map: %s" % e)

        # ============== 发布话题 =====================
        self.obs_map_pub = rospy.Publisher("/obs_map", OccupancyGrid, queue_size=1)
        self.filtered_points_pub = rospy.Publisher("/filtered_points", PointCloud2, queue_size=1)

        # ============== 同步订阅点云和里程计 ==============
        self.pc_sub   = message_filters.Subscriber("/hesai/pandar", PointCloud2)
        self.odom_sub = message_filters.Subscriber("/odom", Odometry)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.pc_sub, self.odom_sub],
            queue_size=1,
            slop=0.1,
            allow_headerless=True
        )
        self.sync.registerCallback(self.synced_callback)

        # ============== 膨胀半径（单位：米 -> 细胞）==================
        self.inflation_radius_m = 2.0
        self.map_res = self.obs_map.info.resolution if self.obs_map.info.resolution else 0.05
        if self.map_res > 0.0:
            self.inflation_radius_cells = int(round(self.inflation_radius_m / self.map_res))
        else:
            self.inflation_radius_cells = 0

        # ============== 新增服务：重置地图功能 =================
        self.reset_map_srv = rospy.Service('reset_map', Empty, self.reset_map_callback)

    def reset_map_callback(self, req):
        """
        回调: 直接从 map_server 获取最新地图，覆盖当前地图，实现“回滚”。
        """
        try:
            latest_map = self.map_client().map
            self.obs_map.data  = np.array(latest_map.data, dtype=np.int8)
            self.obs_map.header = latest_map.header
            self.obs_map.info   = latest_map.info

            rospy.loginfo("Map has been reset to the latest map from map_server.")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to get map during reset: %s" % e)

        return EmptyResponse()

    def synced_callback(self, pc_msg, odom_msg):
        """
        这个回调函数会传入时间上对齐（近似或严格）的点云和里程计消息
        """
        # =========== 1) 点云裁剪，排除车身等 ===========
        cropped_points = []
        for p in read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p
            # 例如: z在[-2, 5.5]之间, 并去除车体周围2m以内的点
            if -2.0 < z < 5.5:
                if abs(x) > 2.0 or abs(y) > 2.0:
                    cropped_points.append((x, y, z))

        if not cropped_points:
            return

        # =========== 2) 根据里程计信息(odom)，做坐标变换 ===========
        odom_x = odom_msg.pose.pose.position.x
        odom_y = odom_msg.pose.pose.position.y
        quat = odom_msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        transformed_points = []
        for (x_car, y_car, z_car) in cropped_points:
            x_odom = odom_x + cos_yaw*x_car - sin_yaw*y_car
            y_odom = odom_y + sin_yaw*x_car + cos_yaw*y_car
            z_odom = z_car
            transformed_points.append((x_odom, y_odom, z_odom))

        # =========== 3) 构建新的点云(只保留 x,y,z) 并发布 ===========
        fields_xyz = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        header = std_msgs.msg.Header()
        header.stamp = pc_msg.header.stamp
        header.frame_id = "odom"

        filtered_cloud = create_cloud(
            header=header,
            fields=fields_xyz,
            points=transformed_points
        )
        self.filtered_points_pub.publish(filtered_cloud)

        # =========== 4) 将点云投影到 OccupancyGrid 上，并进行膨胀 ===========
        map_w = self.obs_map.info.width
        map_h = self.obs_map.info.height
        map_org_x = self.obs_map.info.origin.position.x
        map_org_y = self.obs_map.info.origin.position.y

        # 取出当前地图为 numpy array
        map_data_np = self.obs_map.data  # 已经是 np.array 类型

        # 遍历所有点, 将其对应的栅格(及膨胀区)设置为占据
        for (x_o, y_o, z_o) in transformed_points:
            i_center = int((x_o - map_org_x) / self.map_res)
            j_center = int((y_o - map_org_y) / self.map_res)
            if 0 <= i_center < map_w and 0 <= j_center < map_h:
                self.set_occupied_inflated(map_data_np, i_center, j_center, map_w, map_h)

        # 发布更新后的地图
        self.obs_map.data = map_data_np.tolist()
        self.obs_map.header.stamp = rospy.Time.now()
        self.obs_map_pub.publish(self.obs_map)

        # 重新转回 numpy，方便下一轮更新
        self.obs_map.data = np.array(self.obs_map.data, dtype=np.int8)

    def set_occupied_inflated(self, map_data_np, i_center, j_center, map_w, map_h):
        """
        将(i_center, j_center)以及周围 self.inflation_radius_cells 范围内的格子设置为占据(100)。
        这里用欧几里得距离做膨胀。
        """
        r = self.inflation_radius_cells
        for di in range(-r, r+1):
            for dj in range(-r, r+1):
                dist = np.sqrt(di**2 + dj**2)
                if dist <= r:
                    i_new = i_center + di
                    j_new = j_center + dj
                    if 0 <= i_new < map_w and 0 <= j_new < map_h:
                        idx = i_new + j_new * map_w
                        map_data_np[idx] = 100

if __name__ == '__main__':
    try:
        # 稍等以等待地图服务就绪
        time.sleep(10)
        proj = MapProjection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
