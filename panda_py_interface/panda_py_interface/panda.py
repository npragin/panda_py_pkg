#!/usr/bin/env python3


# Node for interacting with the Panda robot via panda-py
#
# Noah Pragin
#
# This node is used to interact with the Panda robot via panda-py

import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger
from rclpy.parameter import ParameterType

from panda_py_msgs.srv import EndEffectorDeltaPos
from panda_py_msgs.srv import JointPos

import panda_py
from panda_py import libfranka


class PandaInterface(Node):
    def __init__(self):
        super().__init__("panda_interface")

        self.declare_parameter(
            "hostname",
            "192.168.1.11",
            ParameterDescriptor(
                description="The IP address or hostname of the Panda robot",
                type=ParameterType.STRING,
            ),
        )
        self.declare_parameter(
            "gripper_speed",
            0.2,
            ParameterDescriptor(
                description="The speed of the gripper",
                type=ParameterType.DOUBLE,
            ),
        )
        self.declare_parameter(
            "scaling_constant",
            2.5501 ** -1,
            ParameterDescriptor(
                description="The scaling constant for joint position actions",
                type=ParameterType.DOUBLE,
            ),
        )

        self.panda = None
        self.gripper = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node and initialize services."""
        self.get_logger().info("Configuring Panda Interface node...")

        # Create services
        self.stop_service = self.create_service(
            Trigger, "stop", self.stop_callback
        )
        self.move_to_start_service = self.create_service(
            Trigger, "move_to_start", self.move_to_start_callback
        )
        self.end_effector_delta_pos_service = self.create_service(
            EndEffectorDeltaPos,
            "end_effector_delta_pos",
            self.end_effector_delta_pos_callback,
        )
        self.joint_pos_service = self.create_service(
            JointPos,
            "joint_pos",
            self.joint_pos_callback,
        )

        # Initialize robot connection
        try:
            self.panda = panda_py.Panda(self.get_parameter("hostname").value)
            self.gripper = libfranka.Gripper(self.get_parameter("hostname").value)

            if self.panda is None or self.gripper is None:
                self.get_logger().error(
                    "Error initializing Panda robot: Constructors returned None."
                )
                return TransitionCallbackReturn.FAILURE
        except Exception as e:
            self.get_logger().error(f"Error initializing Panda robot: {e}")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Panda Interface node configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node."""
        self.get_logger().info("Activating Panda Interface node...")
        self.get_logger().info("Panda Interface node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info("Deactivating Panda Interface node...")

        # Stop all motion
        try:
            self.panda.stop()
            self.gripper.stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping Panda robot: {e}")
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Panda Interface node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup the node."""
        self.get_logger().info("Cleaning up Panda Interface node...")

        # Destroy services
        self.destroy_service(self.stop_service)
        self.destroy_service(self.move_to_start_service)
        self.destroy_service(self.end_effector_delta_pos_service)
        self.destroy_service(self.joint_pos_service)

        # Cleanup robot connection
        self.panda = None
        self.gripper = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info("Shutting down Panda Interface node...")

        if state.id == State.PRIMARY_STATE_ACTIVE:
            # Stop all motion
            try:
                self.panda.stop()
                self.gripper.stop()
            except Exception as e:
                self.get_logger().error(f"Error stopping Panda robot: {e}")
                return TransitionCallbackReturn.FAILURE
        
        if state.id != State.PRIMARY_STATE_UNCONFIGURED:
            # Destroy services
            self.destroy_service(self.stop_service)
            self.destroy_service(self.move_to_start_service)
            self.destroy_service(self.end_effector_delta_pos_service)
            self.destroy_service(self.joint_pos_service)
            
            # Cleanup robot connection
            self.panda = None
            self.gripper = None

        return TransitionCallbackReturn.SUCCESS
    
    def stop_callback(self, request, response):
        """Stop the robot."""
        if self.get_current_state().id != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        try:
            self.panda.stop()
            self.gripper.stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping Panda robot: {e}")
            return response
        
        self.get_logger().info("Panda robot stopped.")
        return response

    def move_to_start_callback(self, request, response):
        """Move the robot to its start position."""
        if self.get_current_state().id != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        try:
            self.panda.move_to_start()
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot to start: {e}")
            return response

        self.get_logger().info("Panda robot moved to start position.")
        return response

    def end_effector_delta_pos_callback(self, request, response):
        """Move the end effector by a delta position."""
        if self.get_current_state().id != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        pose = self.panda.get_pose()
        pose[0, 3] += request.x
        pose[1, 3] += request.y
        pose[2, 3] += request.z

        try:
            self.panda.move_to_pose(pose)
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot: {e}")
            return response

        self.get_logger().info("Panda robot moved to delta position.")

        new_pose = self.panda.get_pose()
        response.x = new_pose[0, 3]
        response.y = new_pose[1, 3]
        response.z = new_pose[2, 3]

        return response

    def joint_pos_callback(self, request, response):
        """Move the robot to a specific joint position."""
        if self.get_current_state().id != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        joint_pos = request.pos[0:7] * self.get_parameter("scaling_constant").value
        gripper_pos = request.pos[7] * self.get_parameter("scaling_constant").value

        try:
            self.panda.move_to_joint_position(joint_pos)
            self.gripper.move(gripper_pos, self.get_parameter("gripper_speed").value)
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot: {e}")
            return response

        self.get_logger().info("Panda robot moved to position.")

        new_pose = self.panda.get_pose()
        response.x = new_pose[0, 3]
        response.y = new_pose[1, 3]
        response.z = new_pose[2, 3]

        return response


def main(args=None):
    rclpy.init(args=args)

    node = PandaInterface()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
