#!/usr/bin/env python3


# Node for interacting with the Panda robot via panda-py
#
# Noah Pragin
#
# This node is used to interact with the Panda robot via panda-py

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger

from panda_py_msgs.srv import EndEffectorDeltaPos

import panda_py
from panda_py import libfranka


class PandaInterface(Node):
    def __init__(self):
        super().__init__("panda_interface")

        self.declare_parameter(
            "hostname",
            "192.168.1.11",
            ParameterDescriptor(
                description="The IP address or hostname of the Panda robot"
            ),
        )

        self.reinitialize_service = self.create_service(
            Trigger, "reinitialize", self.reinitialize_callback
        )
        self.move_to_start_service = self.create_service(
            Trigger, "move_to_start", self.move_to_start_callback
        )
        self.end_effector_delta_pos_service = self.create_service(
            EndEffectorDeltaPos,
            "end_effector_delta_pos",
            self.end_effector_delta_pos_callback,
        )

        self.panda = None
        self.gripper = None
        try:
            self.panda = panda_py.Panda(self.get_parameter("hostname").value)
            self.gripper = libfranka.Gripper(self.get_parameter("hostname").value)
        except Exception as e:
            self.get_logger().error(
                f"Error initializing Panda robot: {e}\nUse the reinitialize service to try again."
            )
            return

        self.get_logger().info("Panda Interface node initialized.")

    def reinitialize_callback(self, request, response):
        try:
            self.panda = panda_py.Panda(self.get_parameter("panda_hostname").value)
        except Exception as e:
            self.get_logger().error(
                f"Error reinitializing Panda robot: {e}\nUse the reinitialize service to try again."
            )
            return

        self.get_logger().info("Panda Interface node reinitialized.")

        return response

    def move_to_start_callback(self, request, response):
        try:
            self.panda.move_to_start()
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot to start: {e}")
            return

        self.get_logger().info("Panda robot moved to start position.")

        return response

    def end_effector_delta_pos_callback(self, request, response):
        pose = self.panda.get_pose()
        pose[0, 3] += request.delta_pos.x
        pose[1, 3] += request.delta_pos.y
        pose[2, 3] += request.delta_pos.z

        try:
            self.panda.move_to_pose(pose)
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot: {e}")
            return

        self.get_logger().info("Panda robot moved to delta position.")

        new_pose = self.panda.get_pose()
        response.pose.x = new_pose[0, 3]
        response.pose.y = new_pose[1, 3]
        response.pose.z = new_pose[2, 3]

        return response


def main(args=None):
    rclpy.init(args=args)

    node = PandaInterface()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
