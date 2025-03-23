import sys
sys.path.append('/home/lhl/programs/programs/hunter_sim_v1/src/lego/controller/PurePursuit')
import rospy
from nav_msgs.msg import Path, Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
import math
import tf

from geometry_msgs.msg import Twist
import time
import socket
import threading 
import json
import utm
from tf.transformations import quaternion_from_euler
import math
from std_srvs.srv import Empty, EmptyResponse
class myData:
    def __init__(self):
        self.header = {
            "seq": 123,
            "stamp": "2023-05-18T14:12:23Z",
            "frame_id": "base_link"
        }
        self.device_status_id = "id"
        self.pose = {
            "position": {
                "latitude": 0.0,
                "longitude": 0.0,
                "altitude": 0.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            "orientation": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0
            }
        }
        self.twist = {
            "linear": {
                "x": 0.1,
                "y": 0.2,
                "z": 0.3
            },
            "angular": {
                "x": 0.01,
                "y": 0.02,
                "z": 0.03
            }
        }
class task_manager:
    def __init__(self):
        self.pub = rospy.Publisher('filtered_path_marker_array', MarkerArray, queue_size=10)
        self.trans_odom_pub = rospy.Publisher('trans_odom', Odometry, queue_size=10)
        self.sub_path = rospy.Subscriber('/ow/local_path', Path, self.path_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.start_pose_pub = rospy.Publisher('/start_pose', PoseWithCovarianceStamped, queue_size=1)
        self.goal_pose_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_goal = rospy.Subscriber('/move_base_simple/goal1',  PoseStamped, self.goal_callback)
        self.sub_map = rospy.Subscriber('/obs_map', OccupancyGrid, self.map_callback)
        self.sub_map_pub = rospy.Publisher('/sub_map_hybrid_astar', OccupancyGrid,queue_size=1)
        self.left_path_pub = rospy.Publisher("/left_path", Path, queue_size=10)
        self.right_path_pub = rospy.Publisher("/right_path", Path, queue_size=10)
        self.sub_map = rospy.Subscriber('/hunter_se/navsat/fix', NavSatFix, self.navsat_callback)
        self.waypoints_publisher = rospy.Publisher('/waypoints_path', Path, queue_size=1)
        self.service = rospy.Service('return_home', Empty, self.return_home_callback)
        self.waypoints_cnt=0
        self.goal_point=None
        self.start_point=None
        self.searched_path=None
        self.map_data = None
        self.odom_data = None
        self.yaw = 0
        self.path_dir = []
        self.path_is_forward = []
        self.cur_path_is_forward=None
        self.listener = tf.TransformListener()
        self.step = 0
        self.TCP_IP = "127.0.0.1"
        self.TCP_PORT = 50000
        self.BUFFER_SIZE = 1024
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        connected = False
        self.current_xy = (0, 0)
        self.origin_lat = 0
        self.origin_lon = 0
        self.origin_alt = 0
        self.current_lat = 0
        self.current_lon = 0 
        self.current_alt = 0
        self.mydata=myData()
        self.waypoints = []
        print("initial_complete")
        # while not connected:
        #     try:
        #         self.sock.connect((self.TCP_IP, self.TCP_PORT))
        #         connected = True
        #     except socket.error as msg:
        #         rospy.logwarn("Failed to connect to server: %s", msg)
        #         rospy.loginfo("Retrying connection in 5 seconds...")
        #         time.sleep(5)
        # self.send_data_thread =  threading.Thread(target=self.send_data, args=(self.mydata, 10))
        # self.send_data_thread.start()
        # self.receive_data_thread = threading.Thread(target=self.receive_and_process_data)
        # self.receive_data_thread.start()
    def return_home_callback(self,req):
        if self.step==22:
            rospy.loginfo("接收到返航服务调用，开始执行返航流程...")
            self.goal_pose_pub.publish(self.start_point)

            # 在这里添加你的返航逻辑，比如调用飞控模块接口执行返航动作
            # 例如：flight_controller.return_home()
            #rospy.loginfo("返航流程执行完毕。")
            return EmptyResponse()
        else:
            rospy.loginfo("无法执行返航流程...还未到达终点！！")
    def latlon_to_xy(self, latitude, longitude, reference_lat=None, reference_lon=None):
        if reference_lat and reference_lon:
            # If a reference point is provided, calculate the difference in meters
            ref_x, ref_y, ref_zone_num, ref_zone_letter = utm.from_latlon(reference_lat, reference_lon)
            x, y, zone_num, zone_letter = utm.from_latlon(latitude, longitude)
            if ref_zone_num == zone_num and ref_zone_letter == zone_letter:
                return x - ref_x, y - ref_y
        else:
            # Directly convert to UTM
            x, y, _, _ = utm.from_latlon(latitude, longitude)
            return x, y
    def compute_angle_between_points(self, x1, y1, x2, y2):
        """
        Compute the angle (in radians) between two points.
        The angle is measured counter-clockwise from the positive x-axis.
        """
        return math.atan2(y2 - y1, x2 - x1)
    def navsat_callback(self, data):
          # 从 msg 中获取 latitude，longitude 和 altitude 的值
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude

        # 更新 data 中的对应字段
        self.mydata.pose["position"]["latitude"] = lat
        self.mydata.pose["position"]["longitude"] = lon
        self.mydata.pose["position"]["altitude"] = alt
        self.current_lat = lat
        self.current_lon = lon
        self.current_alt = alt
    def send_data(self,data, rate_hz):
        period_sec = 1.0 / rate_hz
        while True:
            message=json.dumps(data.__dict__)
            self.sock.sendall(message.encode())
            time.sleep(period_sec)

    def receive_and_process_data(self):
        path_msg = Path()  # Create a Path message
        path_msg.header.frame_id = "map"  # Assuming "map" as the reference frame, modify if needed

        # Reference point (you can modify this as per your requirements)
        ref_lat, ref_lon = 29.566977, 106.464326

        while not rospy.is_shutdown():
            received_data = self.sock.recv(self.BUFFER_SIZE)
            parsed_data = json.loads(received_data.decode())

            command = parsed_data.get("command", None)
            print(command)
            if command is None:
                continue

            if command == "Gps Initial":
                self.execute_gps_initial(parsed_data)
            elif command == "Some Other Command":
                self.execute_some_other_command(parsed_data)
            elif command == "waypoint":
                self.waypoints = []
                data = parsed_data.get("data", [])
                waypoints_xy = []
                
                for point in data:
                    latitude = point.get("latitude")
                    longitude = point.get("longitude")
                    if latitude is not None and longitude is not None:
                        x, y = self.latlon_to_xy(latitude, longitude, ref_lat, ref_lon)
                        waypoints_xy.append((x, y))
                
                for i, (x, y) in enumerate(waypoints_xy):
                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    
                    # Compute orientation if this is not the last point
                    if i < len(waypoints_xy) - 1:
                        angle = self.compute_angle_between_points(x, y, waypoints_xy[i + 1][0], waypoints_xy[i + 1][1])
                    # For the last point, set the orientation based on the previous point
                    else:
                        angle = self.compute_angle_between_points(waypoints_xy[i - 1][0], waypoints_xy[i - 1][1], x, y)
                    
                    # Convert the angle to a Quaternion and set the orientation
                    q = quaternion_from_euler(0, 0, angle)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    
                    path_msg.poses.append(pose)
                    self.waypoints.append(pose)
                
                # Publish the populated Path message
                self.waypoints_publisher.publish(path_msg)
                self.step=5
                self.waypoints_cnt=0
            else:
                print("Unknown command:", command)

        return path_msg  # If you need to return the Path message after processing
    def execute_gps_initial(self, data):
        # 在这里执行"Gps Initial"命令的相关操作，可以使用data字典中的其他字段
        self.origin_lon=self.current_lon
        self.origin_lat=self.current_lat
        self.origin_alt=self.current_alt
        print(self.origin_lon)
        pass
    def execute_some_other_command(self, data):
        # 在这里执行"Some Other Command"命令的相关操作，可以使用data字典中的其他字段
        pass
    def map_callback(self, data):
        # Store the received map data
        self.map_data = data
        print("map_received")

    def is_path_colliding(self):

        if self.searched_path is None :
            rospy.logwarn("Path  data is not yet available.")
            return False
        if self.map_data is None:
            rospy.logwarn("Path Map data is not yet available.")
            return False
        # Path width in meters
        width = 0.1
        offset = width / 2.0

        left_path = Path()
        right_path = Path()

        left_path.header.frame_id = right_path.header.frame_id = self.searched_path.header.frame_id

        for i in range(len(self.searched_path.poses) - 1):
            start = self.searched_path.poses[i].pose.position
            end = self.searched_path.poses[i + 1].pose.position

            # Compute the direction vector between consecutive points
            dx = end.x - start.x
            dy = end.y - start.y

            # Compute the normal vector
            length = (dx**2 + dy**2)**0.5
            nx = -dy / length
            ny = dx / length

            # Compute points on the parallel lines
            left_point = PoseStamped()
            right_point = PoseStamped()

            left_point.pose.position.x = start.x + offset * nx
            left_point.pose.position.y = start.y + offset * ny

            right_point.pose.position.x = start.x - offset * nx
            right_point.pose.position.y = start.y - offset * ny

            left_path.poses.append(left_point)
            right_path.poses.append(right_point)

            # Publish the paths
            self.left_path_pub.publish(left_path)
            self.right_path_pub.publish(right_path)
            
            if self.line_intersects_obstacles(left_point.pose.position, end, self.map_data.info.origin, self.map_data.info.resolution) \
            or self.line_intersects_obstacles(right_point.pose.position, end, self.map_data.info.origin, self.map_data.info.resolution):
                return True

        

        return False

    def line_intersects_obstacles(self, start, end, origin, resolution):
        start_x = int((start.x - origin.position.x) / resolution)
        start_y = int((start.y - origin.position.y) / resolution)
        end_x = int((end.x - origin.position.x) / resolution)
        end_y = int((end.y - origin.position.y) / resolution)
        
        cells = self.bresenham(start_x, start_y, end_x, end_y)
        
        for (x, y) in cells:
            index = y * self.map_data.info.width + x

            if index < 0 or index >= len(self.map_data.data):
                continue
            
            if self.map_data.data[index] > 0:
                return True

        return False

    def bresenham(self, x1, y1, x2, y2):
        points = []
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        x, y = x1, y1
        sx = -1 if x1 > x2 else 1
        sy = -1 if y1 > y2 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x2:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y2:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x, y))

        return points
    def goal_callback(self,msg):
        if self.step==2:  
            self.path_dir=[]
            self.path_is_forward=[]
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            filter_node.cmd_vel_pub.publish(twist_msg)
            time.sleep(4)
            self.sub_map_pub.publish(self.map_data)

            start_pose_msg = PoseWithCovarianceStamped()
            start_pose_msg.header.stamp = rospy.Time.now()
            start_pose_msg.header.frame_id = "map"  # or "odom" depending on your setup
            start_pose_msg.pose.pose.position.x = self.odom_data.pose.pose.position.x
            start_pose_msg.pose.pose.position.y = self.odom_data.pose.pose.position.y
            start_pose_msg.pose.pose.orientation = self.odom_data.pose.pose.orientation
            self.start_pose_pub.publish(start_pose_msg)

            start_pose_msg = PoseStamped()
            start_pose_msg = msg
            self.goal_pose_pub.publish(start_pose_msg)
            self.step=1
        if self.step==0 or self.step==1:

            self.sub_map_pub.publish(self.map_data)

            start_pose_msg = PoseWithCovarianceStamped()
            start_pose_msg.header.stamp = rospy.Time.now()
            start_pose_msg.header.frame_id = "map"  # or "odom" depending on your setup
            start_pose_msg.pose.pose.position.x = self.odom_data.pose.pose.position.x
            start_pose_msg.pose.pose.position.y = self.odom_data.pose.pose.position.y
            start_pose_msg.pose.pose.orientation = self.odom_data.pose.pose.orientation
            self.start_pose_pub.publish(start_pose_msg)

            start_pose_msg = PoseStamped()
            start_pose_msg = msg
            self.goal_pose_pub.publish(start_pose_msg)
            self.step=1
        if self.step==6:  
            self.path_dir=[]
            self.path_is_forward=[]
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            filter_node.cmd_vel_pub.publish(twist_msg)
            time.sleep(4)
            self.sub_map_pub.publish(self.map_data)

            start_pose_msg = PoseWithCovarianceStamped()
            start_pose_msg.header.stamp = rospy.Time.now()
            start_pose_msg.header.frame_id = "map"  # or "odom" depending on your setup
            start_pose_msg.pose.pose.position.x = self.odom_data.pose.pose.position.x
            start_pose_msg.pose.pose.position.y = self.odom_data.pose.pose.position.y
            start_pose_msg.pose.pose.orientation = self.odom_data.pose.pose.orientation
            self.start_pose_pub.publish(start_pose_msg)

            start_pose_msg = PoseStamped()
            start_pose_msg = msg
            self.goal_pose_pub.publish(start_pose_msg)
            self.step=1
    def angle_between_vectors(self, v1, v2):
        dot_product = v1[0]*v2[0] + v1[1]*v2[1]
        magnitude_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
        magnitude_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
        angle = math.degrees(math.acos(min(1, max(-1, cos_theta))))
        return angle
    def is_point_in_front(self, point):
        """
        Determines if the given point is in front of the vehicle.

        Args:
            point (list): [x, y] coordinates of the point.

        Returns:
            bool: True if the point is in front of the vehicle, False otherwise.
        """
        if self.odom_data is None:
            rospy.logwarn("Odom data is not yet available.")
            return False

        # Get the vehicle's position and orientation from odom_data
        vehicle_position = [self.odom_data.pose.pose.position.x, self.odom_data.pose.pose.position.y]
        quaternion = (
            self.odom_data.pose.pose.orientation.x,
            self.odom_data.pose.pose.orientation.y,
            self.odom_data.pose.pose.orientation.z,
            self.odom_data.pose.pose.orientation.w
        )
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        vehicle_forward = [math.cos(yaw), math.sin(yaw)]

        # Compute the vector to the given point
        vector_to_point = [point[0] - vehicle_position[0], point[1] - vehicle_position[1]]

        # Check if the point is in front or behind
        return sum(i[0] * i[1] for i in zip(vehicle_forward, vector_to_point)) > 0
    def compute_distance_to_goal(self, path_msg):
        # Assuming self.odom_data is already updated and contains the latest position
        if not self.odom_data:
            rospy.logwarn("Odom data is not yet available.")
            return None

        # Extracting the current position from the odom_data
        current_x = self.odom_data.pose.pose.position.x
        current_y = self.odom_data.pose.pose.position.y

        # Extracting the goal position from the path_msg (accessing the 'poses' attribute of the Path message)
        if isinstance(path_msg.poses[-1], list):
            # If path_msg.poses[-1] is a list, we assume the first item of this list is the pose
            goal_x = path_msg.poses[-1][0].pose.position.x
            goal_y = path_msg.poses[-1][0].pose.position.y
        else:
            # Assuming path_msg.poses[-1] directly has a 'pose' attribute
            goal_x = path_msg.poses[-1].pose.position.x
            goal_y = path_msg.poses[-1].pose.position.y

        # Computing the Euclidean distance
        distance = ((current_x - goal_x)**2 + (current_y - goal_y)**2)**0.5

        return distance

    def create_marker(self, x, y, marker_type=Marker.SPHERE, scale=1, color=[1.0, 0.0, 0.0]):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 0.15
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        return marker
    def path_to_json(path_msg):
        # Convert nav_msgs::Path message to a JSON-serializable format
        data = {
            'header': {
                'seq': path_msg.header.seq,
                'stamp': str(path_msg.header.stamp),
                'frame_id': path_msg.header.frame_id
            },
            'poses': [
                {
                    'header': {
                        'seq': pose.header.seq,
                        'stamp': str(pose.header.stamp),
                        'frame_id': pose.header.frame_id
                    },
                    'pose': {
                        'position': {
                            'x': pose.pose.position.x,
                            'y': pose.pose.position.y,
                            'z': pose.pose.position.z
                        },
                        'orientation': {
                            'x': pose.pose.orientation.x,
                            'y': pose.pose.orientation.y,
                            'z': pose.pose.orientation.z,
                            'w': pose.pose.orientation.w
                        }
                    }
                }
                for pose in path_msg.poses
            ]
        }

        return json.dumps(data)
    def path_callback(self, data):
        if self.step==0:
            self.searched_path=data
            self.step=1
            if data.poses:
                # 记录起点和终点
                self.start_point = data.poses[0]
                self.goal_point = data.poses[-1]
                rospy.loginfo("路径起点为: %s", self.start_point)
                rospy.loginfo("路径终点为: %s", self.goal_point)
            else:
                rospy.logwarn("收到的路径为空，无法记录起点和终点")
        if self.step==21:
            self.searched_path=data
            self.step=1
        if self.step==22:
            self.searched_path=data
            self.step=1
    def odom_callback(self, data):
        # 从 msg 中获取位置和方向的值
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        roll, pitch, yaw = tf.transformations.euler_from_quaternion([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ])

        # 更新 data 中的对应字段
        self.mydata.pose["position"]["x"] = x
        self.mydata.pose["position"]["y"] = y
        self.mydata.pose["position"]["z"] = z
        self.mydata.pose["orientation"]["roll"] = roll
        self.mydata.pose["orientation"]["pitch"] = pitch
        self.mydata.pose["orientation"]["yaw"] = yaw

        # 从 msg 中获取线速度和角速度的值
        linear_x = data.twist.twist.linear.x
        linear_y = data.twist.twist.linear.y
        linear_z = data.twist.twist.linear.z
        angular_x = data.twist.twist.angular.x
        angular_y = data.twist.twist.angular.y
        angular_z = data.twist.twist.angular.z

        # 更新 data 中的对应字段
        self.mydata.twist["linear"]["x"] = linear_x
        self.mydata.twist["linear"]["y"] = linear_y
        self.mydata.twist["linear"]["z"] = linear_z
        self.mydata.twist["angular"]["x"] = angular_x
        self.mydata.twist["angular"]["y"] = angular_y
        self.mydata.twist["angular"]["z"] = angular_z
        try:
            (trans, rot) = self.listener.lookupTransform('/map', '/rear_wheel_link', rospy.Time(0))
            x = trans[0]
            y = trans[1]
            data.pose.pose.position.x = trans[0]
            data.pose.pose.position.y = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        self.odom_data=data
        quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
        )

        # Converting quaternion to Euler angles
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        # Printing the yaw angle in degrees
        #rospy.loginfo("Vehicle Yaw: %f degrees", math.degrees(yaw))
        self.yaw = yaw

        pass