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
from sensor_msgs_py import point_cloud2
import zmq
import pickle
import numpy as np

from panda_py_msgs.srv import JointPos, EndEffectorDeltaPos
from panda_py_msgs.msg import JointPos as JointPosMsg

class PolicyNode(Node):
    def __init__(self):
        super().__init__("policy")

        self.declare_parameter('action_space', 'joint_pos')
        self.declare_parameter('zmq_server_address', 'tcp://flip4.engr.oregonstate.edu:96002')
        self.declare_parameter('goal_pos', [0.05, 0.05, 0.15])

        # Initialize ZeroMQ context and socket
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        zmq_server_address = self.get_parameter('zmq_server_address').value
        self.zmq_socket.connect(zmq_server_address)
        self.get_logger().info(f"Connected to ZeroMQ server at {zmq_server_address}")

        self.action_client = None
        self.create_action_client()

        self.image_sub = self.create_subscription(
            Image,
            "image",
            self.image_callback,
            10,
        )
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            "pruned_pointcloud",
            self.point_cloud_callback,
            10,
        )
        self.trajectory_client = self.create_service(
            Trigger,
            "trajectory",
            self.trajectory_callback,
        )
        self.joint_pos_sub = self.create_subscription(
            JointPosMsg,
            "panda_state/joint_pos",
            self.joint_pos_callback,
            10,
        )

        self.joint_pos = None

        # TODO: create_action_client() should only be called once after converting to a lifecycle node
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
        elif action_space == 'joint_delta_pos':
            self.action_client = self.create_client(
                JointPos,
                "joint_delta_pos",
            )
        elif action_space == 'end_effector_delta_pos':
            self.action_client = self.create_client(
                EndEffectorDeltaPos,
                "end_effector_delta_pos",
            )
        else:
            self.get_logger().error(f"Invalid action space: {action_space}")

    def joint_pos_callback(self, msg):
        self.get_logger().info(f"Received joint position: {msg.pos}")
        self.joint_pos = msg.pos

    def image_callback(self, msg):
        raise NotImplementedError("Image callback not implemented")

    def point_cloud_callback(self, msg):
        self.get_logger().info(
            f"Received a {type(msg)} message. Calling motion service with arbitrary joint position."
        )

        if self.joint_pos is None:
            self.get_logger().error("Joint position not received yet")
            return

        pc = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z', 'rgb'])
        
        zmq_data = {
            'message': pickle.dumps(pc),
            'goal_pos': self.get_parameter('goal_pos').value,
            'joint_pos': self.joint_pos
        }

        try:
            zmq_message = pickle.dumps(zmq_data)
            self.zmq_socket.send(zmq_message)
            self.get_logger().info(f"Sent data to ZeroMQ server")
            
            response = self.zmq_socket.recv()
            response_data = pickle.loads(response)
            self.get_logger().info(f"Received response from server: {response_data}")
            
        except Exception as e:
            self.get_logger().error(f"Error sending data via ZeroMQ: {e}")

        self.get_logger().info(f"Absolute joint angles after executing this action: {np.array(self.joint_pos) + np.array(response_data)}")
        input("Press Enter to execute action...")

        if self.action_client is None:
            self.get_logger().error("Action client not initialized")
            return

        elif self.get_parameter('action_space').value == 'joint_pos' or self.get_parameter('action_space').value == 'joint_delta_pos':
            action = list(response_data)

            action_request = JointPos.Request()
            action_request.pos = action

            if self.get_parameter('action_space').value == 'joint_delta_pos':
                action = self.joint_delta_pos_clip_scale(action)

        elif self.get_parameter('action_space').value == 'end_effector_delta_pos':
            action_request = EndEffectorDeltaPos.Request()
            action_request.x = 0.0
            action_request.y = 0.0
            action_request.z = 0.0

        self.get_logger().info(f"Calling action service with action {action_request}")
        self.action_client.call_async(action_request)

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
    
    def joint_delta_pos_clip_scale(self, joint_delta_pos):
        """
        Clips and scales the joint pos delta to match Maniskill3 single action limits
        """
        high = 0.1
        low = -0.1

        result = []
        for i in range(len(joint_delta_pos)):
            clipped_action = max(min(joint_delta_pos[i], high), low)
            scaled_action = 0.5 * (high + low) + 0.5 * (high - low) * clipped_action
            result.append(scaled_action)

        return result




def main(args=None):
    rclpy.init(args=args)

    node = PolicyNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
