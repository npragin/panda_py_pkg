import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.intrinsics_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.intrinsics_callback,
            10
        )

        self.bridge = CvBridge()
        self.intrinsics = None
        self.distortion_coefficients = None
        
        self.get_logger().info('Camera Calibrator Node Initialized')

    def intrinsics_callback(self, msg):
        self.get_logger().info(f"Received Camera Info: {msg.K}")
        self.intrinsics = msg.K
        self.distortion_coefficients = msg.D
        self.destroy_subscription(self.intrinsics_subscription)

    def image_callback(self, msg):
        self.get_logger().info(f"Received Image: width={msg.width}, height={msg.height}")

        if self.intrinsics is None or self.distortion_coefficients is None:
            self.get_logger().warn("No intrinsics or distortion coefficients received yet.")
            return

        transformation_matrix = self.estimate_poses(self.bridge.imgmsg_to_cv2(msg, 'bgr8'))

        if transformation_matrix is not None:
            self.get_logger().info(f"Estimated transformation matrix: {transformation_matrix}")

            # Publish transformation matrix
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'table'

            translation = transformation_matrix[0:3, 3]
            rotation = transformation_matrix[0:3, 0:3]

            rotation = R.from_matrix(rotation)
            quat = rotation.as_quat()

            t.transform.translation.x = float(translation[0])
            t.transform.translation.y = float(translation[1])
            t.transform.translation.z = float(translation[2])

            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]

            self.tf_static_broadcaster.sendTransform(t)

    def estimate_poses(self, color_image):
        """
        Estimates poses for the given color image.
        Returns transformation matrix.
        """

        camera_matrix = np.array(
            [
                [self.intrinsics[0], 0, self.intrinsics[2]],
                [0, self.intrinsics[4], self.intrinsics[5]],
                [0, 0, 1],
            ]
        )
        dist_coeffs = np.array(self.distortion_coefficients)

        CHECKERBOARD = (9, 8)
        square_size = 0.05
        objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        x, y = np.meshgrid(range(CHECKERBOARD[0]), range(CHECKERBOARD[1]))
        # Stack X and Y coordinates, but flip X ??? to point upward
        objp[:, :2] = np.stack((x[:, ::-1].flatten(), y.flatten()), axis=1)
        objp *= square_size

        print("Estimating poses and creating masked images...")

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCornersSB(
            gray_image, CHECKERBOARD, flags=cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
        )

        if ret:
            # If we find the checkerboard
            # Proceed with pose estimation
            ret, rvec, tvec = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)
            if ret:
                # If we can estimate the pose from the checkerboard
                R_board_to_camera, _ = cv2.Rodrigues(rvec)
                R_camera_to_board = R_board_to_camera.T
                t_camera = -np.matrix(R_camera_to_board) * np.matrix(tvec)

                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = R_camera_to_board
                transformation_matrix[:3, 3] = t_camera.flatten()

                return transformation_matrix
            else:
                # If we can't estimate the pose from the checkerboard
                print(f"Pose estimation failed for image")
        else:
            # If we can't find the checkerboard
            print(f"Checkerboard not found in frame")

        return None


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
