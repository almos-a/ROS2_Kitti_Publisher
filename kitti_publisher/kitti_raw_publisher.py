import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField, Imu
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster
import os
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Header
import struct
import transforms3d.euler as euler
import transforms3d.quaternions as quat_utils
from datetime import datetime

class KittiRawPublisher(Node):
    def __init__(self):
        super().__init__('kitti_raw_publisher')

        # Parameters
        self.declare_parameter('base_path', '/home/almos/slam_ws/data/2011_09_26/2011_09_26_drive_0022_sync/')
        self.declare_parameter('playback_rate', 1.0)  # 1x speed

        base_path = self.get_parameter('base_path').get_parameter_value().string_value
        self.playback_rate = self.get_parameter('playback_rate').get_parameter_value().double_value

        self.cam0_pub = self.create_publisher(Image, '/kitti/image_00', 10)
        self.cam1_pub = self.create_publisher(Image, '/kitti/image_01', 10)
        self.cam2_pub = self.create_publisher(Image, '/kitti/image_02', 10)
        self.cam3_pub = self.create_publisher(Image, '/kitti/image_03', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/kitti/velodyne', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/kitti/pose', 10)
        self.imu_pub = self.create_publisher(Imu, '/kitti/imu', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # Paths
        self.cam_paths = {
            0: os.path.join(base_path, 'image_00', 'data'),
            1: os.path.join(base_path, 'image_01', 'data'),
            2: os.path.join(base_path, 'image_02', 'data'),
            3: os.path.join(base_path, 'image_03', 'data'),
        }
        self.velo_path = os.path.join(base_path, 'velodyne_points', 'data')
        self.oxts_path = os.path.join(base_path, 'oxts', 'data')
        self.timestamp_file = os.path.join(base_path, 'oxts', 'timestamps.txt')

        # Load timestamps
        self.timestamps = self.load_timestamps(self.timestamp_file)

        self.current_idx = 0
        self.paused = False

        # Timer
        self.timer = self.create_timer(0.1 / self.playback_rate, self.timer_callback)

        # Playback service
        self.srv = self.create_service(SetBool, 'toggle_playback', self.toggle_callback)

        self.get_logger().info(f"KITTI RAW Publisher initialized with playback rate {self.playback_rate}x")

    def load_timestamps(self, filepath):
        stamps = []
        with open(filepath, 'r') as f:
            lines = f.readlines()
            for line in lines:
                timestamp_str = line.strip()
                if '.' in timestamp_str:
                    # Truncate after 6 digits of microseconds
                    date_part, subsec_part = timestamp_str.split('.')
                    subsec_part = subsec_part[:6]  # keep only 6 digits
                    fixed_timestamp = f"{date_part}.{subsec_part}"
                else:
                    fixed_timestamp = timestamp_str
                dt = datetime.strptime(fixed_timestamp, '%Y-%m-%d %H:%M:%S.%f')

                time_msg = rclpy.time.Time.from_msg(self.datetime_to_ros(dt))
                stamps.append(time_msg)
        return stamps

    def datetime_to_ros(self, dt):
        from builtin_interfaces.msg import Time
        unix_time = dt.timestamp()
        sec = int(unix_time)
        nsec = int((unix_time - sec) * 1e9)
        return Time(sec=sec, nanosec=nsec)

    def timer_callback(self):
        if self.paused:
            return

        if self.current_idx >= len(self.timestamps):
            self.get_logger().info("Finished publishing all frames.")
            return

        timestamp = self.timestamps[self.current_idx]

        # Publish Cameras
        for idx, pub in zip(range(4), [self.cam0_pub, self.cam1_pub, self.cam2_pub, self.cam3_pub]):
            img_file = os.path.join(self.cam_paths[idx], f"{self.current_idx:010d}.png")
            if os.path.exists(img_file):
                img = cv2.imread(img_file, cv2.IMREAD_COLOR)
                if img is not None:
                    msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                    msg.header.stamp = timestamp.to_msg()
                    msg.header.frame_id = f"camera_{idx}"
                    pub.publish(msg)

        # Publish Velodyne
        velo_file = os.path.join(self.velo_path, f"{self.current_idx:010d}.bin")
        if os.path.exists(velo_file):
            points = np.fromfile(velo_file, dtype=np.float32).reshape(-1, 4)
            cloud_msg = self.create_pointcloud2(points, timestamp)
            self.lidar_pub.publish(cloud_msg)

        # Publish IMU and Pose
        oxts_file = os.path.join(self.oxts_path, f"{self.current_idx:010d}.txt")
        if os.path.exists(oxts_file):
            imu_msg, pose_msg, transform_msg = self.parse_oxts(oxts_file, timestamp)
            self.imu_pub.publish(imu_msg)
            self.pose_pub.publish(pose_msg)
            self.tf_broadcaster.sendTransform(transform_msg)

        self.current_idx += 1

    def create_pointcloud2(self, points, timestamp):
        header = Header()
        header.stamp = timestamp.to_msg()
        header.frame_id = "velodyne"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * points.shape[0]
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        return msg

    def parse_oxts(self, filepath, timestamp):
        data = np.loadtxt(filepath)
        lat, lon, alt, roll, pitch, yaw = data[0:6]
        vf, vl, vu = data[6:9]  # velocity
        af, al, au = data[9:12]  # acceleration
        wf, wl, wu = data[12:15]  # angular velocity

        # Pose
        pose = PoseStamped()
        pose.header.stamp = timestamp.to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = lon  # For simplicity (you should later convert properly to ENU!)
        pose.pose.position.y = lat
        pose.pose.position.z = alt

        rot = euler.euler2quat(roll, pitch, yaw, axes='sxyz')
        pose.pose.orientation.w = rot[0]
        pose.pose.orientation.x = rot[1]
        pose.pose.orientation.y = rot[2]
        pose.pose.orientation.z = rot[3]

        # IMU
        imu = Imu()
        imu.header.stamp = timestamp.to_msg()
        imu.header.frame_id = "imu"
        imu.linear_acceleration.x = af
        imu.linear_acceleration.y = al
        imu.linear_acceleration.z = au
        imu.angular_velocity.x = wf
        imu.angular_velocity.y = wl
        imu.angular_velocity.z = wu
        imu.orientation = pose.pose.orientation

        # Transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp.to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "imu"
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation = pose.pose.orientation

        return imu, pose, tf_msg

    def toggle_callback(self, request, response):
        self.paused = not self.paused
        response.success = True
        response.message = f"Playback {'paused' if self.paused else 'resumed'}."
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KittiRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
