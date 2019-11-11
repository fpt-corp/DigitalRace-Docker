#!/usr/bin/env python

import time

import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage, Image

import cv2
import numpy as np

from utils.param import Param
#import algorithms.lane.segmentation as segmentation
from algorithms import lane, sign, obstacle
from control import car_control, lcd_control


class DiRaNode:
    def __init__(self, param):
        self.runnable = True
        self.go_sub = rospy.Subscriber("/ss_status", Bool, callback=self.callback_ss, queue_size=1)

        self.param = param

        # Algorithm classes
        self.obstacle_detector = obstacle.detection.DepthProcessor()
        self.sign_detector = sign.detection.SignDetector(self.param.model_sign_path)
        self.lane_segmenter = lane.segmentation.AutoEncoder(self.param.model_lane_path)

        # Controllers
        self.car_controller = car_control.CarController(self.param)
        self.lcd_controller = lcd_control.LCDController(self.param)

        # Counter for fps limitation
        self.cc_counter = 0
        self.sd_counter = 0
        self.od_counter = 0
        self.bt_counter = 0

        self.sign = (0, (0, 0, 0, 0))
        self.danger_zone = (0, 0)

        self.bridge = CvBridge()

        rospy.Subscriber("/team1/camera/rgb/compressed", CompressedImage,
                         callback=self.callback_control, queue_size=1)
        rospy.Subscriber("/team1/camera/rgb/compressed", CompressedImage,
                         callback=self.callback_detect_sign, queue_size=1)
        rospy.Subscriber("/team1/camera/depth/compressed", Image,
                         callback=self.callback_detect_obstacle, queue_size=1)

        self.fps_counter_test = 0
        self.fps_timer = time.time()

    def callback_ss(self, data):
        self.runnable = data.data

    def callback_control(self, data):
        try:
            self.cc_counter += 1
            if self.cc_counter % 2 == 0:
                self.cc_counter = 0

                np_arr = np.fromstring(data.data, np.uint8)
                image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                # NOTE: image_np.shape = (160, 320, 3)
                image_np = cv2.resize(image_np, (320, 160))
                # Drive
                self.car_controller.control(image_np, self.sign, self.runnable, self.danger_zone)

            self.bt_counter += 1
            if self.bt_counter % 10 == 0:
                self.bt_counter = 0

                self.lcd_controller.check_button()
        except CvBridgeError as e:
            print(e)

    def callback_detect_sign(self, data):
        try:
            self.sd_counter += 1
            if self.sd_counter % 3 == 0:
                self.sd_counter = 0
                np_arr = np.fromstring(data.data, np.uint8)
                image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                # image_np = self.bridge.imgmsg_to_cv2(data)
                image_np = cv2.resize(image_np, (320, 240))
                image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

                self.sign = self.sign_detector.detect_sign(image_np)

                if self.sign[0] != 0:
                    print("Sign: ", self.sign)
                # cv2.imshow("sign_detection", img_out)
                # cv2.waitKey(1)

                # always get the max P of list

        except CvBridgeError as e:
            print(e)

    def callback_detect_obstacle(self, data):

        try:
            self.od_counter += 1
            if self.od_counter % 2 == 0:
                self.od_counter = 0

                self.fps_counter_test += 1
                if time.time() - self.fps_timer > 1:
                    print(self.fps_counter_test)
                    self.fps_counter_test = 0
                    self.fps_timer = time.time()

                # image_np = self.dc.process_compressedDepth(data)
                image_np = self.bridge.imgmsg_to_cv2(data)

                image_np = cv2.resize(image_np, (320, 240))
                image_np = image_np[100:, :]

                # cv2.imshow('img_depth', image_np)
                # cv2.waitKey(1)
                # timer = time.time()
                self.danger_zone = self.obstacle_detector.combine(image_np * 10)
                # print(self.danger_zone)
                # timer = time.time() - timer
                # print("time: ", timer)
                # print(time.time() - timer)

                # print(self.danger_zone)

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    param = Param()

    roslib.load_manifest(param.pkg_name)
    rospy.init_node(param.node_name, anonymous=True)

    dira_node = DiRaNode(param)
    rospy.spin()
