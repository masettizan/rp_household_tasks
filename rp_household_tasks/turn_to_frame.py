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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rp_houshold_tasks_msgs.action import Frame
from math import atan2

# access to frame transformations between 'camera_link' and a given 'frame_param'
# returns information on how to transform 'camera_link' to face given 'frame_param'
class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')

        # declare and acquire frame parameter
        self.frame_param = self.declare_parameter(
          'frame', 'base_link').get_parameter_value().string_value

        self.get_logger().info(f'frame: {self.frame_param}')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # if a valid transformation between 'camera_link' and 'frame_param' is found
        self.position = None

        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.on_timer, callback_group=ReentrantCallbackGroup())

    # get difference between 'camera_link' and the 'frame_param'
    def on_timer(self):
        # store frame names in variables that will be used to compute transformations
        target_frame = 'camera_link'
        source_frame = self.frame_param

        try:
            # look up for the transformation between target_frame and source_frame
            position = self.buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame, 
                time=Time(),
                timeout=Duration(seconds=1.0)
            )
            # assign tranformation to global variable
            self.position = position
            self.get_logger().info(f'diff: {position}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not tranform "{target_frame}" to "{source_frame}": {ex}'
            )
    
    # return position information
    # if no position is found within 5 seconds, stop looking and return None
    def get_position(self, total_time = 0.0):
        total_time = total_time
        while (total_time < 5):
            if self.position is not None:
                return self.position.transform
            
            total_time += 0.1
            time.sleep(.1)

        self.get_logger().error(
            f'Could not get tranformation from "camera_link" to "{self.frame_param}" because we ran out of time'
        )
        return None
    
    # change value of 'frame_param'
    def change_goal_frame(self, new_goal_frame):
        self.frame_param = new_goal_frame

# turn the camera to the frame 
class TurnToFrame(Node):

    def __init__(self):
        super().__init__('turn_to_frame')
        # initialize transformation listeners
        self.joint_state = JointState()
        self.frame_position = FrameListener()

        # subscribe to 'joint_state' updates
        self._joint_state_subscriber = self.create_subscription(
            JointState, 
            "/joint_states", 
            self.joint_state_callback, 1
        )
        self._joint_state_subscriber # avoid uncalled variable error
        
        # connect to action client to move camera
        self._follow_joint_trajectory_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/stretch_controller/follow_joint_trajectory',
        )
        # check if action client connected to it's action server
        server_reached = self._follow_joint_trajectory_action_client.wait_for_server(timeout_sec=60.0)
        if not server_reached: 
            # if unable to connect after 60 secs give up and exit program
            self.get_logger().error('Unable to connect to action server, time out was exceeded')
            sys.exit()

        # spin multiple Nodes to run code in parallel
        exe = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        exe.add_node(self.frame_position)
        exe.add_node(self)
        exe_thread = threading.Thread(target=exe.spin, daemon=True)
        exe_thread.start()
    
        # boolean value : has the program been requested to stop
        self.stopped = False
        
        # establish action server to start/stop/change frame
        self._turn_to_frame_action_server = ActionServer(
            self,
            Frame,
            '/turn_to_frame_status',
            self.turn_to_frame_callback
        )
        
        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    # set info from joint_state subscriber to global variable
    def joint_state_callback(self, joint_state):
        self.joint_state = joint_state

    # action server callback, handle requests
    def turn_to_frame_callback(self, goal_handle):
        # set global values to given request values
        self.frame_position.change_goal_frame(goal_handle.request.frame)
        self.stopped = goal_handle.request.stopped

        self.get_logger().info(f"Changing frame to {self.frame_position.frame_param}")
        # request handled successfully
        goal_handle.succeed()
        result = Frame.Result()

        return result

    # move camera to given pan and tilt positions 
    def move_to(self, joint_state: JointState, camera_pan: float, camera_tilt: float):
        goal_position = FollowJointTrajectory.Goal()

        pan_joint_index = joint_state.name.index('joint_head_pan')
        tilt_joint_index = joint_state.name.index('joint_head_tilt')

        # trajectory point motion
        goal_point = JointTrajectoryPoint()

        duration = Duration(seconds=0.1)
        goal_point.time_from_start = duration.to_msg()

        # goal location for pan and tilt 
        pan_joint_goal = camera_pan + joint_state.position[pan_joint_index]
        tilt_joint_goal = camera_tilt + joint_state.position[tilt_joint_index]

        goal_point.positions = [pan_joint_goal, tilt_joint_goal]
        
        # set goal position
        goal_position.trajectory.joint_names = ['joint_head_pan', 'joint_head_tilt']
        goal_position.trajectory.points = [goal_point]
        goal_position.trajectory.header.stamp = self.get_clock().now().to_msg()

        #request movement
        self._follow_joint_trajectory_action_client.send_goal_async(goal_position)

    # main: move camera to target every 0.1 seconds
    def on_timer(self):
        # check if the program was request to stop through action server
        if self.stopped:
            return

        # get transform between 'camera_link' and 'frame_param'
        position = self.frame_position.get_position()
        # if we haven't seen TF in 5 seconds, don't look
        if position is None: 
            return 

        # calculate camera pan movement -- is z or y needed?
        camera_pan = -1 * atan2(position.translation.z, position.translation.x)
        # calculate camera tilt movement -- is y or z needed?
        camera_tilt = atan2(position.translation.y, position.translation.x)

        # make small moves to goal
        camera_pan = np.sign(camera_pan) * np.min([.1, abs(camera_pan)])
        camera_tilt = np.sign(camera_tilt) * np.min([.1, abs(camera_tilt)])

        # assign global variable to local variable to avoid conflicts
        state = self.joint_state
        # if the state is active
        if (state is not None):
            self.move_to(state, camera_pan, camera_tilt)
        

def main():
    time.sleep(5) # Allow time for realsense camera to boot up before this node becomes active
    rclpy.init()
    node = TurnToFrame()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()



