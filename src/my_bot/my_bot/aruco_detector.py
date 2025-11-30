import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

def quaternion_from_matrix(matrix):
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info(f"Aruco Detector Node Started. OpenCV Version: {cv2.__version__}")
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10)
        self.publisher_ = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()
        
    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info("Received Camera Info")
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info("Waiting for camera info...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # self.get_logger().info("Detecting markers...")
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            self.get_logger().info(f"Detected {len(ids)} markers. Estimating pose...")
            # Estimate pose for each marker
            # Marker size is 0.2m (defined in SDF)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.2, self.camera_matrix, self.dist_coeffs)
            self.get_logger().info("Pose estimated.")

            for i in range(len(ids)):
                # Publish the first detected marker pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link_optical' # Camera frame

                pose_msg.pose.position.x = tvecs[i][0][0]
                pose_msg.pose.position.y = tvecs[i][0][1]
                pose_msg.pose.position.z = tvecs[i][0][2]

                # Convert rvec to quaternion
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                # Create 4x4 matrix
                mat = np.eye(4)
                mat[:3, :3] = rotation_matrix
                
                quat = quaternion_from_matrix(mat)
                
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                self.publisher_.publish(pose_msg)
                break # Only handle one marker for now

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
