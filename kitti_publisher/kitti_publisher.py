import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped
import transforms3d.quaternions as quat_utils
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import struct
from std_srvs.srv import SetBool

SEMANTIC_COLOR_MAP = {
    0: (0, 0, 0),          # unlabeled
    1: (255, 0, 0),        # car
    10: (255, 100, 100),   # bicycle
    11: (255, 150, 150),   # motorcycle
    13: (255, 200, 0),     # bus
    15: (255, 150, 50),    # truck
    18: (255, 255, 0),     # person
    20: (128, 64, 128),    # road
    30: (244, 35, 232),    # sidewalk
    40: (70, 70, 70),      # building
    50: (107, 142, 35),    # vegetation
    70: (153, 153, 153),   # pole
    80: (220, 220, 0),     # traffic sign
}

class KittiPublisher(Node):
    def __init__(self):
        super().__init__('kitti_publisher')

        self.image_pub = self.create_publisher(Image, 'kitti/image', 10)
        self.lidar_pub = self.create_publisher(PointCloud2, 'kitti/velodyne', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'kitti/pose', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.frame_id = 0
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.kitti_seq_path = '/dataset/sequences/08/' 
        self.image_path = os.path.join(self.kitti_seq_path, 'image_0')
        self.lidar_path = os.path.join(self.kitti_seq_path, 'velodyne')
        self.label_path = os.path.join(self.kitti_seq_path, 'label') #passed in wrong label dir name to skip semantic labelling
        # to re-enable semantic labels pass corects 'labels' dir
        self.poses = self.load_poses(os.path.join(self.kitti_seq_path, 'poses.txt'))

        self.paused = False
        self.srv = self.create_service(SetBool, 'toggle_playback', self.toggle_callback)

    def timer_callback(self):
        # ---------- IMAGE ----------
        img_file = os.path.join(self.image_path, f"{self.frame_id:06d}.png")
        if not os.path.exists(img_file):
            self.get_logger().info("No more KITTI frames found.")
            return

        cv_image = cv2.imread(img_file, cv2.IMREAD_GRAYSCALE)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera"
        self.image_pub.publish(ros_image)

        # ---------- VELODYNE ----------
        lidar_file = os.path.join(self.lidar_path, f"{self.frame_id:06d}.bin")
        label_file = os.path.join(self.label_path, f"{self.frame_id:06d}.label")
        point_cloud = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

        if os.path.exists(label_file):
            labels = np.fromfile(label_file, dtype=np.uint32).reshape(-1)
            semantic_labels = labels & 0xFFFF  # Semantic is lower 16 bits
            #self.get_logger().info(f"Semantic Labels: {semantic_labels[:10]}")

            colors = np.array([SEMANTIC_COLOR_MAP.get(l, (0, 0, 0)) for l in semantic_labels])
            cloud_msg = self.create_colored_pointcloud2(point_cloud, colors)
        else:
            cloud_msg = self.create_pointcloud2(point_cloud)

        self.lidar_pub.publish(cloud_msg)

        self.frame_id += 1

        if self.frame_id < len(self.poses):
            # PoseStamped message
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"

            T = self.poses[self.frame_id]
            pose.pose.position.x = T[0, 3]
            pose.pose.position.y = T[1, 3]
            pose.pose.position.z = T[2, 3]

            rot_mat = T[:3, :3]
            quat = quat_utils.mat2quat(rot_mat)  # [w, x, y, z]
            pose.pose.orientation.w = quat[0]
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]

            self.pose_pub.publish(pose)

            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = "velodyne"
            transform.transform.translation.x = T[0, 3]
            transform.transform.translation.y = T[1, 3]
            transform.transform.translation.z = T[2, 3]
            transform.transform.rotation.w = quat[0]
            transform.transform.rotation.x = quat[1]
            transform.transform.rotation.y = quat[2]
            transform.transform.rotation.z = quat[3]

            self.tf_broadcaster.sendTransform(transform)

        if self.paused:
            return


    def create_pointcloud2(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "velodyne"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        data = points.astype(np.float32).tobytes()

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats x 4 bytes each
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = data

        return msg
    
    def create_colored_pointcloud2(self, points, colors):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "velodyne"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=16, datatype=PointField.UINT32, count=1),
        ]

        cloud_data = []
        for i in range(points.shape[0]):
            x, y, z, intensity = points[i]
            r, g, b = colors[i]
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]  # packed as BGRA
            cloud_data.append(struct.pack('ffffI', x, y, z, intensity, rgb))

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 20  # 4 floats + 1 uint32 = 20 bytes
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = b''.join(cloud_data)

        return msg

    def load_poses(self, pose_file):
        poses = []
        with open(pose_file, 'r') as f:
            for line in f.readlines():
                T = np.fromstring(line, sep=' ').reshape(3, 4)
                pose = np.eye(4)
                pose[:3, :4] = T
                poses.append(pose)
        return poses
    
    def toggle_callback(self, request, response):
        self.paused = not self.paused
        response.success = True
        response.message = f"Playback {'paused' if self.paused else 'resumed'}."
        self.get_logger().info(response.message)
        return response



def main(args=None):
    rclpy.init(args=args)
    node = KittiPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
