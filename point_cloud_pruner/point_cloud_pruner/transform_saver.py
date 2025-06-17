import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_sensor_msgs import do_transform_cloud

class TransformSaver(Node):
    def __init__(self):
        super().__init__('transform_saver')
        
        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to point cloud topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.point_cloud_callback,
            10)
        
        self.get_logger().info('Transform Saver Node Initialized')
        
    def point_cloud_callback(self, msg: PointCloud2):
        try:
            # Look up the transform from camera to table
            transform = self.tf_buffer.lookup_transform(
                'table',
                msg.header.frame_id,
                rclpy.time.Time())
            
            transformed_cloud = do_transform_cloud(msg, transform)
            points = np.array(list(pc2.read_points(transformed_cloud, skip_nans=True, field_names=("x", "y", "z", "rgb"))))
            
            np.save('transformed_points.npy', points)
            self.get_logger().info('Successfully saved transformed points to transformed_points.npy')
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 