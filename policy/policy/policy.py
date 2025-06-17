#!/usr/bin/env python3


# Node for RL Policy Inference
#
# Noah Pragin
#
# Node that subscribes to a point cloud or image topic and publishes a policy action

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_srvs.srv import Trigger

from panda_py_msgs.srv import JointPos, EndEffectorDeltaPos


class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy")

        self.declare_parameter('action_space', 'joint_pos')

        self.action_client = None
        self.create_action_client()

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
        self.trajectory_client = self.create_service(
            Trigger,
            "trajectory",
            self.trajectory_callback,
        )

        while self.action_client is None or not self.action_client.wait_for_service(timeout_sec=1.0):
            self.create_action_client()
            self.get_logger().info("Waiting for action client...")

        self.trajectory = [
            [
                -0.00042694016155853207,
                -0.7830637130261214,
                0.00014937929567725332,
                -2.3578286012815375,
                -8.568492018192321e-05,
                1.5713621334238594,
                0.7878076746938086,
                0.07983556389808655,
            ],
            [
                -0.0008775664328694073,
                -0.5001480132012025,
                -0.023384826150941285,
                -2.3823757146285143,
                -0.0011275570529988125,
                1.772412006768944,
                0.7858626272980684,
                0.07983425259590149,
            ],
        ]
        self.trajectory_index = 0

        self.get_logger().info("Policy node initialized")
    
    def create_action_client(self):
        action_space = self.get_parameter('action_space').value

        if action_space == 'joint_pos':
            self.action_client = self.create_client(
                JointPos,
                "joint_pos",
            )
        elif action_space == 'joint_pos_delta':
            self.action_client = self.create_client(
                JointPos,
                "joint_pos_delta",
            )
        elif action_space == 'end_effector_delta_pos':
            self.action_client = self.create_client(
                EndEffectorDeltaPos,
                "end_effector_delta_pos",
            )
        else:
            self.get_logger().error(f"Invalid action space: {action_space}")

    def callback(self, msg):
        self.get_logger().info(
            f"Received a {type(msg)} message. Calling motion service with arbitrary joint position."
        )

        if self.action_client is None:
            self.get_logger().error("Action client not initialized")
            return
        elif self.get_parameter('action_space').value == 'joint_pos' or self.get_parameter('action_space').value == 'joint_pos_delta':
            joint_pos = JointPos.Request()
            joint_pos.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            self.action_client.call_async(joint_pos)
        elif self.get_parameter('action_space').value == 'end_effector_delta_pos':
            end_effector_delta_pos = EndEffectorDeltaPos.Request()
            end_effector_delta_pos.x = 0.0
            end_effector_delta_pos.y = 0.0
            end_effector_delta_pos.z = 0.0

            self.action_client.call_async(end_effector_delta_pos)

    def trajectory_callback(self, request, response):
        self.get_logger().info("Received trajectory request")

        if self.action_client is None:
            self.get_logger().error("Action client not initialized")
            return

        if self.get_parameter('action_space').value != 'joint_pos':
            self.get_logger().error("Hardcoded trajectory only works with joint_pos action space")
            return

        request = JointPos.Request()

        request.pos = self.trajectory[self.trajectory_index]
        self.trajectory_index = (self.trajectory_index + 1) % len(self.trajectory)

        self.get_logger().info(f"Calling motion service with joint position {request.pos}")

        self.action_client.call_async(request)

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    node = PolicyNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
