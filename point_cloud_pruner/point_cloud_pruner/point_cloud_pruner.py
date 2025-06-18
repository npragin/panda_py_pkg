import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class PointCloudPruner(Node):
    def __init__(self):
        super().__init__('pointcloud_pruner')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/transformed_points',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/pruned_pointcloud', 10)

        self.max_depth = 1.5  # depth threshold in meters

        self.get_logger().info('PointCloud Pruner Initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f"Received PointCloud2: width={msg.width}, height={msg.height}")
        # Convert incoming PointCloud2 to numpy array
        raw_points = list(pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "rgb")))
        if not raw_points:
            self.get_logger().warn("No valid points received.")
            return

        # Filter based on depth
        pruned = [p for p in raw_points if p[1] <= self.max_depth]
        if not pruned:
            self.get_logger().warn("All points pruned — skipping publish.")
            return
        
        cloud_out = pc2.create_cloud(msg.header, msg.fields, pruned)

        # Publish pruned point cloud
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
