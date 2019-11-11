#!/usr/bin/env python

import math
import time

import rospy
from std_msgs.msg import Float32, String, Bool

import cv2
import numpy as np

from algorithms import lane

class CarController:
    def __init__(self, param):
        rospy.init_node(param.node_name, anonymous=True)
        self.pub_speed = rospy.Publisher("/team1/set_speed", Float32, queue_size=1)
        self.pub_steer = rospy.Publisher("/team1/set_angle", Float32, queue_size=1)

        self.param = param
        self.lane_segmenter = lane.segmentation.AutoEncoder(self.param.model_lane_path)

        self.init_moving = True
        self.time = 0
        self.current_speed = 0

        self.h, self.w = 160, 320
        self.sign_type = 0
        self.last_detected = 0

        self.fps = 0
        self.fps_timer = time.time()
        self.sharp = False

    def control(self, img, sign, runnable, danger_zone):
        """
        Control the car, publish steer_angle and speed to the engine
        :param x: Coordinates of both sidelines of the road
        :param sign: Sign type
        :param runnable: True if the car can run
        :param danger_zone:
        :return: none
        """
        now = time.time()
        steer_angle = self.cal_steer_angle(img, sign, danger_zone)
        speed = 0

        if not rospy.is_shutdown():
            if not runnable:
                self.init_moving = True
                self.current_speed = 0
                steer_angle = 0
            elif self.init_moving:
                self.init_moving = False
                self.time = now
            elif now - self.time > self.param.delay_time and self.current_speed == 0:
                self.current_speed = self.param.min_speed

            if self.current_speed != 0:
                speed = max(self.param.min_speed,
                            self.current_speed - self.param.speed_decay * (
                                    self.param.base_speed - self.param.min_speed) * abs(steer_angle ** 2) / (
                                    self.param.max_steer_angle ** 2))

                if self.current_speed < self.param.base_speed:
                    self.current_speed += 0.5

            # Publish to the engine
            self.pub_speed.publish(speed)
            self.pub_steer.publish(steer_angle)

    def cal_steer_angle(self, img, sign, danger_zone):
        """
        Calculate steer angle
        :param x: Coordinates of both sidelines of the road
        :param sign: Sign type
        :param danger_zone:
        :return: steer_angle
        """
        # fps counter
        self.fps += 1
        now = time.time()
        if now - self.fps_timer > 1:
            print("fps ", self.fps)
            self.fps_timer = time.time()
            self.fps = 0

        if sign[0] != 0:
            self.sign_type = sign[0]
            self.last_detected = time.time()

        self.lane_segmenter.roi = 0.7
        points = self.lane_segmenter.get_points(img)

        middle_pos = (points[0] + points[1]) / 2

        rate = 0.55
        self.sharp = False
        if points[1] - points[0] < 170:
            if middle_pos < 120:
                self.sign_type = 2
                #self.last_detected = time.time()
                
                print("left")
                self.sharp = True
            elif middle_pos > 200:
                self.sign_type = 1
                #self.last_detected = time.time()
                print("right")
                self.sharp = True
        # If still in turning time
        if now - self.last_detected < self.param.turning_time or self.sharp == True:
            self.lane_segmenter.roi = 0.9
            points = self.lane_segmenter.get_points(img)
            middle_pos = (points[0] + points[1]) / 2

            if self.sign_type == 1:
                #middle_pos = rate * points[1] + (1 - rate) * middle_pos
                middle_pos = points[1] - 50
            elif self.sign_type == 2:
                #middle_pos = rate * points[0] + (1 - rate) * middle_pos
                middle_pos = points[0] + 50
        #else:
        #    self.lane_segmenter.roi = 0.5

        print("middle_pos: {}, x: {}, {}, y: {}".format(middle_pos, points[0], points[1], points[2]))
        print(danger_zone)
        # Avoid obstacles
        if danger_zone != (0, 0):

            # 2 objects
            if danger_zone[0] == -1:
                print("2 obstacles")
                middle_pos = danger_zone[1]
            # Single object
            else:
                center_danger_zone = int((danger_zone[0] + danger_zone[1]) / 2)

                if danger_zone[0] < middle_pos < danger_zone[1]:
                    # Obstacle's on the right
                    if middle_pos < center_danger_zone:
                        print("on the right")
                        middle_pos = danger_zone[0]
                    # Left
                    else:
                        print("on the left")
                        middle_pos = danger_zone[1]

        cv2.line(img, (int(middle_pos), self.h / 2), (self.w / 2, self.h), (255, 0, 0), 2)

        # Distance between middle position and car_position
        distance_x = middle_pos - self.w / 2

        distance_y = self.h - points[2]
        #distance_y = self.h * 0.5
        #if now - self.last_detected < self.param.turning_time:
            #distance_y = self.h * 0.3

        # Angle to middle position
        steer_angle = math.atan(float(distance_x) / distance_y) * 180 / math.pi

        return steer_angle
