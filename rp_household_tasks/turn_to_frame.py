#! /usr/bin/env python3

import sys
import rclpy
import threading
import rclpy.executors
import time
import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient, ActionServer
from rclpy.duration import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rp_houshold_tasks_msgs.action import Frame
from math import atan2

# from stretch_core.keyboard import KBHit

# gives position information about base_link
class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')

        frame_param = 'camera_link'
        self.declare_parameter('frame', frame_param)
        self.frame_param = self.get_parameter('frame').get_parameter_value().string_value

        self.get_logger().info(f'Frame: {self.frame_param}')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.position = None #change none
        # self.kb_input = KBHit()

        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        try:
            now = Time()
            position = self.buffer.lookup_transform(
                target_frame='camera_link',
                source_frame=self.frame_param, #switching 
                time=now,
                timeout=Duration(seconds=1.0)
            )
            self.position = position
            self.get_logger().info(
                f'diff: {position}'
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not tranform "camera_link" to "{self.frame_param}": {ex}'
            )
        
        # self.change_goal_frame()
        
    def get_position(self, total_time = 0.0):
        total_time = 0
        while (total_time < 5):
            if self.position is not None:
                return self.position.transform
            
            total_time += 0.1
            time.sleep(.1)

        self.get_logger().error(
            f'Could not tranform "camera_link" to "{self.frame_param}" and ran out of time'
        )
        return None
        
    def change_goal_frame(self, new_goal):
        self.get_logger().info(
                f'NEW GFOAL \n\n\n\n\n'
            )
        self.frame_param = new_goal
            # if self.kb_input.kbhit(): # Returns True if any key pressed
            #     activation = self.kb_input.getch() # Enter == "\n"
            #     self.frame_param = activation


class TurnToFrame(Node):

    def __init__(self):
        super().__init__('turn_to_frame')
        
        self.joint_state = JointState()

        # keep communicating updated camera_link position
        self.frame_position = FrameListener()
        # subscribe to joint state updates
        self.joint_state_subscriber = self.create_subscription(
            JointState, 
            "/joint_states", 
            self.callback, 1
        )
        self.joint_state_subscriber
        # connect to action client to move camera
        self._follow_joint_trajectory_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/stretch_controller/follow_joint_trajectory'
        )
        server_reached = self._follow_joint_trajectory_action_client.wait_for_server(timeout_sec=60.0)

        exe = rclpy.executors.MultiThreadedExecutor()
        exe.add_node(self.frame_position)
        exe.add_node(self)
        exe_thread = threading.Thread(target=exe.spin, daemon=True)
        exe_thread.start()


        if not server_reached:
            self.get_logger().error('Unable to connect to action server, time out was exceeded')
            sys.exit()

        self.stopped = False
        
        self._turn_to_frame_action_server = ActionServer(
            self,
            Frame,
            '/turn_to_frame_status',
            self.turn_to_frame_callback
        )
        
        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def callback(self, joint_state):
        self.joint_state = joint_state

    def turn_to_frame_callback(self, goal_handle):
        self.get_logger().info(f"executing turn to frame callback {goal_handle.request}")
        self.frame_position.change_goal_frame(goal_handle.request.frame)
        self.stopped = goal_handle.request.stopped

        self.get_logger().info(f"Changing frame to {self.frame_position.frame_param}")
        goal_handle.succeed()
        result = Frame.Result()
        return result



    def on_timer(self):
        if self.stopped:
            return
        
        goal_position = FollowJointTrajectory.Goal()
        position = self.frame_position.get_position()
        # self.get_logger().info("WE IN MIN\n\n\n\n\n\n")


        if position is None:
            return # dont look: haven't seen TF in 5 seconds

        # calculate camera pan movement -- is z or y needed?
        camera_pan = -1 * atan2(position.translation.z, position.translation.x)

        # calculate camera tilt movement -- is y or z needed?
        camera_tilt = atan2(position.translation.y, position.translation.x)

        # make small moves to goal
        camera_pan = np.sign(camera_pan) * np.min([.1, abs(camera_pan)])
        camera_tilt = np.sign(camera_tilt) * np.min([.1, abs(camera_tilt)])

        state = self.joint_state
        # if the state is active
        if (state is not None):
            # self.get_logger().info(f"state: {state}")
            pan_joint_index = state.name.index('joint_head_pan')
            tilt_joint_index = state.name.index('joint_head_tilt')

            # trajectory point motion
            goal_point = JointTrajectoryPoint()

            duration = Duration(seconds=0.1)
            goal_point.time_from_start = duration.to_msg()

            # goal location for pan and tilt 
            pan_joint_goal = camera_pan + state.position[pan_joint_index]
            tilt_joint_goal = camera_tilt + state.position[tilt_joint_index]

            goal_point.positions = [pan_joint_goal, tilt_joint_goal]
            
            # set goal position
            goal_position.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
            goal_position.trajectory.points = [goal_point]
            goal_position.trajectory.header.stamp = self.get_clock().now().to_msg()
            
            # self.get_logger().info(f"joint info: {goal_point}")

            #request movement
            self._follow_joint_trajectory_action_client.send_goal_async(goal_position)
            # self.get_logger().info(f"joint info: {pan_joint_goal}, {tilt_joint_goal}")
        

def main():
    time.sleep(5) # Allows time for realsense camera to boot up before this node becomes active
    rclpy.init()
    node = TurnToFrame()
    # node.main()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()



