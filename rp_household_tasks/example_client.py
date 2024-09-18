#!/usr/bin/env python3 

import rclpy
import time
import sys

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rp_houshold_tasks_msgs.action import Frame

# example action client class
class ActionClientExample(Node):

    def __init__(self):
        super().__init__('action_client_example')
        # create action client
        self._action_client = ActionClient(self, Frame, '/turn_to_frame_status')

        # check if action client connected to it's action server
        server_reached = self._action_client.wait_for_server(timeout_sec=60.0)
        if not server_reached: 
            # if unable to connect after 60 secs give up and exit program
            self.get_logger().error('Unable to connect to action server, time out was exceeded')
            sys.exit()

    # send request to action server
    def send_goal(self):
        goal_msg = Frame.Goal()
        goal_msg.stopped = False
        goal_msg.frame = 'link_gripper_finger_right' 

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # check that request to action server succeeded
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Result: Success')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = ActionClientExample()
    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()