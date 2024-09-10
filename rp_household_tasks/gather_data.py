#! /usr/bin/env python3


import sys
import termios
import tty

from numpy import select
from rclpy.node import Node
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
import rclpy
from tf2_ros import TransformException
import threading
import rclpy.executors
import time
import csv
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# gives position information about base_link
class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.position = None #change none

        time_period = 0.1 #seconds
        self.timer = self.create_timer(time_period, self.on_timer)

    def on_timer(self):
        try:
            now = Time()
            position = self.buffer.lookup_transform(
                target_frame='map',
                source_frame='base_link', #switvhing 
                time=now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.position = position
            # self.get_logger().info(
            #     f'WE OUT HERE FRAME LISTENING: x: {position.transform.translation.x} y: {position.transform.translation.y}'
            # )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not tranform "map" to "base_link": {ex}'
            )
        
    def get_position(self, total_time = 0.0):
        if total_time >= 5:
            self.get_logger().error(
                f'Could not tranform "map" to "base_link" and ran out of time'
            )
            return
        if self.position is None:
            self.get_logger().info(
                f'waiting'
            )
            total_time += .1
            time.sleep(.1)
            return self.get_position(total_time)
        else:
            return self.position.transform

# robot class
class GatherData(Node):

    def __init__(self, node_name='gather_data', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        
        self.navigator = BasicNavigator()

        # keep communicating updated base_link position
        self.base_link_position = FrameListener()
        

        exe = rclpy.executors.MultiThreadedExecutor()
        exe.add_node(self.base_link_position)
        exe_thread = threading.Thread(target= exe.spin, daemon=True)
        exe_thread.start()

        self.set_locations()

        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/camera/color/image', self.get_image, 10)
        self.image_subscriber

        activation = input("Press 'Enter' to start program.")
        if (activation == ""):
            #TODO: MAKE ACTRIVATED BY KEY PRESS: 'Enter'
            self.set_initial_pose() # can assume same initial position

            # Wait for navigation to fully activate, since autostarting nav2
            self.navigator.waitUntilNav2Active() #does this need intial pose to be set?

    # extract information from csv file containing all possible robot goal locations in EXP 120
    def set_locations(self):
        locations_path = '/home/hello-robot/ament_ws/src/rp_household_tasks/rp_household_tasks/locations.csv'
        self.locations = {}
        #store locations in dict of tuples --> location : (x, y)
        with open(locations_path, 'r', newline='') as file:
            csv_reader = csv.DictReader(file, delimiter=':')
            for row in csv_reader:
                self.locations[row['location']] = (row['position_x'], row['position_y'])

    # set intial pose to where the robot is currently
    def set_initial_pose(self):
        position = self.locations['A']
        self.get_logger().info(
                f'initial pose: {position} x: {position[0]} y: {position[1]}'
            )
        
        initial_pose = self.get_pose(
            position[0],
            position[1]
        )
                                    
        self.navigator.setInitialPose(initial_pose)

    # return PoseStamped instance of pose given
    def get_pose(self, pos_x=0.0, pos_y=0.0, rot_z=0.0, rot_w=1.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()

        pose.pose.position.x = float(pos_x)
        pose.pose.position.y = float(pos_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = float(rot_z)
        pose.pose.orientation.w = float(rot_w)
        return pose
    
    def a_to_b(self):
        pose_b = self.get_pose(
            pos_x=self.locations["B"][0], 
            pos_y=self.locations["B"][1]
        )
        pose_b.pose.orientation.w = 1.0

        self.navigator.goToPose(pose_b)
        while not self.navigator.isTaskComplete():
            position = self.base_link_position.get_position()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("yipee successfully got to position B")
        else:
            self.get_logger().error('something went wrong -_-')

    def b_to_c(self):
        pose_c = self.get_pose(
            pos_x=self.locations["C"][0], 
            pos_y=self.locations["C"][1]
        )
        pose_c.pose.orientation.w = 1.0

        self.navigator.goToPose(pose_c)
        while not self.navigator.isTaskComplete():
            position = self.base_link_position.get_position()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("yipee successfully got to position C")
        else:
            self.get_logger().error('something went wrong -_-')

    def get_image(self, data):
        path = "/home/hello-robot/ament_ws/src/rp_household_tasks/rp_household_tasks/"
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def main(self):

        # A -> B
        self.a_to_b()

        #scan with camera
        cv2.imwrite('/home/hello-robot/ament_ws/src/rp_household_tasks/rp_household_tasks/image.png', cv_image)

        # B -> C
        self.b_to_c()

def main():
    try:
        time.sleep(1)

        rclpy.init()
        data_gatherer_bot = GatherData()
        data_gatherer_bot.main() # do i call main before spin???

        rclpy.spin(data_gatherer_bot)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()