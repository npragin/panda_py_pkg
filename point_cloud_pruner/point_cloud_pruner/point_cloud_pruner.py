import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
from tf2_sensor_msgs import do_transform_cloud

class PointCloudPruner(Node):
    def __init__(self):
        super().__init__('pointcloud_pruner')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, '/pruned_pointcloud', 10)

        self.max_depth = 1.5  # depth threshold in meters

        self.get_logger().info('PointCloud Pruner Initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f"Received PointCloud2: width={msg.width}, height={msg.height}")

        # Transform point cloud to table frame
        transform = self.tf_buffer.lookup_transform(
            "table",
            msg.header.frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0),
        )

        if transform is None:
            self.get_logger().warn("Could not transform point cloud to table frame.")
            cloud = msg
        else:
            cloud = do_transform_cloud(msg, transform)

        # Convert incoming PointCloud2 to numpy array
        raw_points = list(pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z", "rgb")))
        if not raw_points:
            self.get_logger().warn("No valid points received.")
            return

        # Filter based on z
        pruned = [p for p in raw_points if p[2] <= self.max_depth]
        if not pruned:
            self.get_logger().warn("All points pruned — skipping publish.")
            return
        
        self.get_logger().info(f"Publishing {len(pruned)} pruned points with RGB.")

        # Define PointField structure for xyz + rgb
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Create new PointCloud2 message
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        cloud_out = pc2.create_cloud(header, fields, pruned)

        # Publish pruned point cloud
        self.publisher.publish(cloud_out)
        self.get_logger().info("Published pruned RGB point cloud.")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPruner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
