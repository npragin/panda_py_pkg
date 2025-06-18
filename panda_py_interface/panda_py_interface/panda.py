#!/usr/bin/env python3


# Node for interacting with the Panda robot via panda-py
#
# Noah Pragin
#
# This node is used to interact with the Panda robot via panda-py

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State as StateMsg
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Trigger
from rclpy.parameter import ParameterType
import numpy as np

from panda_py_msgs.srv import EndEffectorDeltaPos
from panda_py_msgs.srv import JointPos
import panda_py
from panda_py import libfranka


class PandaInterface(LifecycleNode):
    def __init__(self):
        super().__init__("panda_interface")

        self.declare_parameter(
            "hostname",
            "192.168.1.11",
            ParameterDescriptor(
                description="The IP address or hostname of the Panda robot",
                type=ParameterType.PARAMETER_STRING,
            ),
        )
        self.declare_parameter(
            "gripper_speed",
            0.2,
            ParameterDescriptor(
                description="The speed of the gripper",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )
        self.declare_parameter(
            "scaling_constant",
            2.5501**-1,
            ParameterDescriptor(
                description="The scaling constant for joint position actions",
                type=ParameterType.PARAMETER_DOUBLE,
            ),
        )

        self.panda = None
        self.gripper = None

        self.status = StateMsg.PRIMARY_STATE_UNCONFIGURED

        self.get_logger().info("Panda Interface node initialized")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure the node and initialize services."""
        self.get_logger().info("Configuring Panda Interface node...")

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
        
        # Create services
        self.stop_service = self.create_service(Trigger, "stop", self.stop_callback)
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
        self.joint_delta_pos_service = self.create_service(
            JointPos,
            "joint_delta_pos",
            self.joint_delta_pos_callback,
        )
        self.state_timer = self.create_timer(0.1, self.state_callback)
        self.joint_pos_publisher = self.create_publisher(
            JointPos,
            "panda_state/joint_pos",
            1,
        )
        self.joint_vel_publisher = self.create_publisher(
            JointPos,
            "panda_state/joint_vel",
            1,
        )

        self.status = StateMsg.PRIMARY_STATE_INACTIVE
        self.get_logger().info("Panda Interface node configured.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate the node."""
        self.get_logger().info("Activating Panda Interface node...")

        self.status = StateMsg.PRIMARY_STATE_ACTIVE
        self.get_logger().info("Panda Interface node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate the node."""
        self.get_logger().info("Deactivating Panda Interface node...")

        # Stop all motion
        try:
            self.panda.stop_controller()
            self.gripper.stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping Panda robot: {e}")
            return TransitionCallbackReturn.FAILURE

        self.status = StateMsg.PRIMARY_STATE_INACTIVE
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
        self.destroy_service(self.joint_delta_pos_service)
        self.destroy_timer(self.state_timer)
        self.destroy_publisher(self.joint_pos_publisher)
        self.destroy_publisher(self.joint_vel_publisher)

        # Cleanup robot connection
        self.panda = None
        self.gripper = None

        self.status = StateMsg.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info("Shutting down Panda Interface node...")

        if state.id == State.PRIMARY_STATE_ACTIVE:
            # Stop all motion
            try:
                self.panda.stop_controller()
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
            self.destroy_service(self.joint_delta_pos_service)
            self.destroy_timer(self.state_timer)
            self.destroy_publisher(self.joint_pos_publisher)
            self.destroy_publisher(self.joint_vel_publisher)

            # Cleanup robot connection
            self.panda = None
            self.gripper = None

        self.status = StateMsg.PRIMARY_STATE_UNCONFIGURED
        return TransitionCallbackReturn.SUCCESS

    def stop_callback(self, request, response):
        """Stop the robot."""
        if self.status != StateMsg.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        try:
            self.panda.stop_controller()
            self.gripper.stop()
        except Exception as e:
            self.get_logger().error(f"Error stopping Panda robot: {e}")
            return response

        self.get_logger().info("Panda robot stopped.")
        return response

    def move_to_start_callback(self, request, response):
        """Move the robot to its start position."""
        if self.status != StateMsg.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        try:
            self.panda.move_to_start()
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot to start: {e}")
            return response

        self.get_logger().info("Panda robot moved to start position.")
        return response
    
    def state_callback(self):
        """Publish the current state of the robot."""
        state = self.panda.get_state()

        q = state.q
        gripper_width = self.gripper.readOnce().width
        dq = state.dq
        # NOTE: libfranka does not provide gripper velocity
        gripper_velocity = 0

        self.joint_pos_publisher.publish(JointPos(pos=q + [gripper_width]))
        self.joint_vel_publisher.publish(JointPos(pos=dq + [gripper_velocity]))

    def end_effector_delta_pos_callback(self, request, response):
        """Move the end effector by a delta position."""
        if self.status != StateMsg.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        pose = self.panda.get_pose()
        pose[0, 3] += request.x * self.get_parameter("scaling_constant").value
        pose[1, 3] += request.y * self.get_parameter("scaling_constant").value
        pose[2, 3] += request.z * self.get_parameter("scaling_constant").value

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
        if self.status != StateMsg.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        self.get_logger().info(f"Got request with joint position {request.pos}")

        requested_joint_pos = np.array(request.pos[0:7])

        joint_pos = requested_joint_pos
        gripper_pos = request.pos[7]

        self.get_logger().info(f"Sending joint position {joint_pos} and gripper position {gripper_pos} to the Panda robot")

        try:
            self.panda.move_to_joint_position(joint_pos)
            self.gripper.move(gripper_pos, self.get_parameter("gripper_speed").value)
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot: {e}")
            return response

        self.get_logger().info("Panda robot moved to position.")
        return response

    def joint_delta_pos_callback(self, request, response):
        """Move the robot to a specific joint position delta."""
        if self.status != State.PRIMARY_STATE_ACTIVE:
            self.get_logger().error("Panda Interface node is not active.")
            return response

        current_joint_pos = np.array(self.panda.get_state().q)
        current_gripper_pos = self.gripper.readOnce().width
        requested_joint_pos = np.array(request.pos[0:7])

        joint_pos = (
            current_joint_pos
            + requested_joint_pos * self.get_parameter("scaling_constant").value
        )
        gripper_pos = (
            current_gripper_pos
            + request.pos[7] * self.get_parameter("scaling_constant").value
        )

        try:
            self.panda.move_to_joint_position(joint_pos)
            self.gripper.move(gripper_pos, self.get_parameter("gripper_speed").value)
        except Exception as e:
            self.get_logger().error(f"Error moving Panda robot: {e}")
            return response

        self.get_logger().info("Panda robot moved to position delta.")
        return response


def main(args=None):
    rclpy.init(args=args)

    node = PandaInterface()
    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
