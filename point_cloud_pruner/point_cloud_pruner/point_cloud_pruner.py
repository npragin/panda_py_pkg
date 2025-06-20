import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudPruner(Node):
    def __init__(self):
        super().__init__('pointcloud_pruner')

        self.declare_parameter('max_x', np.inf)
        self.declare_parameter('min_x', -np.inf)
        self.declare_parameter('max_y', np.inf)
        self.declare_parameter('min_y', -np.inf)
        self.declare_parameter('max_z', np.inf)
        self.declare_parameter('min_z', -np.inf)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/transformed_points',
            self.listener_callback,
            1)
        self.publisher = self.create_publisher(PointCloud2, '/pruned_pointcloud', 1)

        self.get_logger().info('PointCloud Pruner Initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f"Received PointCloud2: width={msg.width}, height={msg.height}")

        raw_points = pc2.read_points_numpy(msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))

        # Filter based on x, y, z bounds
        pruned = raw_points[(raw_points[:, 0] >= self.get_parameter('min_x').value) &
                            (raw_points[:, 0] <= self.get_parameter('max_x').value) &
                            (raw_points[:, 1] >= self.get_parameter('min_y').value) &
                            (raw_points[:, 1] <= self.get_parameter('max_y').value) &
                            (raw_points[:, 2] >= self.get_parameter('min_z').value) &
                            (raw_points[:, 2] <= self.get_parameter('max_z').value)]
        
        if len(pruned) == 0:
            self.get_logger().warn("All points pruned — skipping publish.")
            return
        
        cloud_out = pc2.create_cloud(msg.header, msg.fields, pruned)

        self.publisher.publish(cloud_out)
        self.get_logger().info(f"Published {len(pruned)} pruned points.")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPruner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
