#!/usr/bin/env python3


# Manager node for the RL inference system on the Panda robot.
#
# Noah Pragin
#
# This node is responsible for starting and stopping the RL inference system on the Panda robot.

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from std_srvs.srv import Trigger
import time


class ManagerNode(Node):
    def __init__(self):
        super().__init__("manager")

        self.startup_service = self.create_service(
            Trigger, "startup_system", self.startup_callback
        )
        self.shutdown_service = self.create_service(
            Trigger, "shutdown_system", self.shutdown_callback
        )

        # Nodes will be configured, activated, and shut down in the order they are listed here.
        self.managed_nodes = ["panda_interface"]
        self.lifecycle_change_state_clients = {}
        self.lifecycle_get_state_clients = {}
        for node in self.managed_nodes:
            self.create_lifecycle_client(node)

        self.get_logger().info("Manager node initialized")

    def create_lifecycle_client(self, node_name):
        self.lifecycle_change_state_clients[node_name] = self.create_client(
            ChangeState,
            f"/{node_name}/change_state",
        )
        self.lifecycle_get_state_clients[node_name] = self.create_client(
            GetState,
            f"/{node_name}/get_state",
        )

        while not self.lifecycle_change_state_clients[node_name].wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(f"Waiting for {node_name} to be ready...")
            time.sleep(1.0)

        while not self.lifecycle_get_state_clients[node_name].wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(f"Waiting for {node_name} to be ready...")
            time.sleep(1.0)

    async def startup_callback(self, request, response):
        self.get_logger().info("Starting system...")

        # Configure managed nodes
        for node_name, client in self.lifecycle_change_state_clients.items():
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            result = await client.call_async(request)
            if result.success:
                self.get_logger().info(f"{node_name} configured")
            else:
                self.get_logger().error(f"Failed to configure {node_name}")
                response.success = False
                response.message = f"Failed to configure {node_name}"
                return response

        # Wait to activate nodes until calibration is complete
        while not self.is_calibration_complete():
            self.get_logger().info("Waiting for calibration to complete...")
            time.sleep(1.0)

        # Activate managed nodes
        for node_name, client in self.lifecycle_change_state_clients.items():
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_ACTIVATE
            result = await client.call_async(request)
            if result.success:
                self.get_logger().info(f"{node_name} activated")
            else:
                self.get_logger().error(f"Failed to activate {node_name}")
                response.success = False
                response.message = f"Failed to activate {node_name}"
                return response

        response.success = True
        response.message = "System started"
        return response

    async def shutdown_callback(self, request, response):
        self.get_logger().info("Shutting down system...")

        for node_name, client in self.lifecycle_change_state_clients.items():
            request = ChangeState.Request()

            curr_state = await self.lifecycle_get_state_clients[node_name].call_async(
                GetState.Request()
            )
            if curr_state.current_state.id == State.PRIMARY_STATE_ACTIVE:
                request.transition.id = Transition.TRANSITION_ACTIVE_SHUTDOWN
            elif curr_state.current_state.id == State.PRIMARY_STATE_INACTIVE:
                request.transition.id = Transition.TRANSITION_INACTIVE_SHUTDOWN
            elif curr_state.current_state.id == State.PRIMARY_STATE_UNCONFIGURED:
                request.transition.id = Transition.TRANSITION_UNCONFIGURED_SHUTDOWN
            else:
                self.get_logger().error(
                    f"Node {node_name} in unhandled state: {curr_state.current_state.label}, {curr_state.current_state.id}"
                )
                response.success = False
                response.message = f"Node {node_name} in unhandled state: {curr_state.current_state.label}, {curr_state.current_state.id}"
                return response

            result = await client.call_async(request)
            if not result.success:
                self.get_logger().error(f"Failed to shutdown {node_name}")
                response.success = False
                response.message = f"Failed to shutdown {node_name}"
                return response

        response.success = True
        response.message = "System shut down"
        return response

    def is_calibration_complete(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "camera_depth_optical_frame",
                "table",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            if transform is None:
                self.get_logger().error("No transform found.")
                return False

        except Exception as e:
            self.get_logger().error(f"Error looking up transform: {e}")
            return False

        return True


def main(args=None):
    rclpy.init(args=args)

    node = ManagerNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
