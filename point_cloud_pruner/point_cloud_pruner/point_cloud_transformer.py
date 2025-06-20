import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from panda_py_msgs.msg import TransformMatrix
from std_msgs.msg import Header

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('point_cloud_transformer')

        self.declare_parameter("additional_x_translation", 0.0)
        self.declare_parameter("additional_y_translation", 0.0)
        self.declare_parameter("additional_z_translation", 0.0)

        self.transform_subscriber = self.create_subscription(
            TransformMatrix,
            '/transform_matrix',
            self.transform_callback,
            10
        )

        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.point_cloud_callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/transformed_points',
            10
        )

        self.transform = None

        self.get_logger().info('Point Cloud Transformer Node Initialized')

    def transform_callback(self, msg: TransformMatrix):
        transform_matrix = np.array([row.data for row in msg.rows])

        maniskill_transform_matrix = np.array([
            [0, -1, 0],
            [1,  0, 0],
            [0,  0, 1]
        ])

        transform_matrix[:3, :3] = maniskill_transform_matrix @ transform_matrix[:3, :3]
        
        # Apply additional translations to the transform matrix
        additional_translation = np.array([
            self.get_parameter("additional_x_translation").value,
            self.get_parameter("additional_y_translation").value,
            self.get_parameter("additional_z_translation").value
        ])
        transform_matrix[:3, 3] += additional_translation
        
        self.transform = transform_matrix
        self.get_logger().info('Received new transform matrix')
        
    def point_cloud_callback(self, msg: PointCloud2):
        if self.transform is None:
            self.get_logger().warn('No transform received yet, skipping point cloud')
            return

        try:
            # Extract points from point cloud
            points = np.array(pc2.read_points_numpy(msg, skip_nans=True, field_names=("x", "y", "z", "rgb")))

            # Extract rotation and translation from transform
            R = self.transform[:3, :3]
            t = self.transform[:3, 3]

            # Transform points
            transformed_points = points.copy()
            transformed_points[:, :3] = (R @ points[:, :3].T).T + t
            
            # Create PointCloud2 message
            header = Header()
            header.stamp = msg.header.stamp
            header.frame_id = 'table'
            cloud_out = pc2.create_cloud(header, msg.fields, transformed_points)
            
            self.publisher.publish(cloud_out)
            self.get_logger().info('Successfully published transformed points')
            
        except Exception as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 