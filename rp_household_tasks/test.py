#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class image_converter(Node):
    def __init__(self):
        super().__init__("image_converter")
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)

    def callback(self,data):
        # try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # except:
        #     cv_image = self.bridge.imgmsg_to_cv2(data)

        print(type(cv_image)) # numpy
        print(cv_image.shape) 
        print(cv2.imwrite('/home/hello-robot/ament_ws/src/rp_household_tasks/rp_household_tasks/image.png', cv_image))
        
        
        
        #TODO: HOW THE HELL DOES THIS WOPEK ???? WHY NO WORK ??
        # img = cv2.imread('/home/hello-robot/ament_ws/src/rp_household_tasks/rp_household_tasks/image.png', 1)
        # cv2.imshow('image', img)
        

def main(args=None):
    rclpy.init()
    ic = image_converter()
    try:
        rclpy.spin_once(ic)
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
'''topic, msg, timestamp = bag.read_messages(topics=['/color/image_raw'])
    
if topic == '/color/image_raw':
    color_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imwrite('depth_{}.jpg'.format(count), color_img)

'''