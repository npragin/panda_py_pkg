#!/usr/bin/env python3


# Node for RL Policy Inference
#
# Noah Pragin
#
# Node that subscribes to a point cloud or image topic and publishes a policy action

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from panda_py_msgs.srv import JointPos


class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy")

        self.joint_pos_delta_service = self.create_client(
            JointPos,
            "joint_pos_delta",
        )
        self.image_sub = self.create_subscription(
            Image,
            "image",
            self.callback,
            10,
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            "point_cloud",
            self.callback,
            10,
        )

        while not self.joint_pos_delta_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for joint_pos_delta service...")

        self.get_logger().info("Policy node initialized")

    def callback(self, msg):
        self.get_logger().info(f"Received a {type(msg)} message. Calling motion service with arbitrary joint position.")

        joint_pos = JointPos.Request()
        joint_pos.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joint_pos.gripper = 0.0

        self.joint_pos_delta_service.call_async(joint_pos)


def main(args=None):
    rclpy.init(args=args)

    node = PolicyNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
