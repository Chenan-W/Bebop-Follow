#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020/11/6 下午7:21
# @Author : Chenan_Wang
# @File : follow_ros_py.py
# @Software: CLion

import rospy
import cv2 as cv
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class image_converter:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # 定义结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
        # print kernel

        height, width = cv_image.shape[0:2]
        screen_center = width / 2
        screen_center_h = height / 2
        offset = 50
        offset_h = 30

        lower_b = (75, 43, 46)
        upper_b = (110, 255, 255)
        hsv_frame = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_frame, lower_b, upper_b)
        mask2 = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask3 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)
        cv.imshow("mask", mask3)

        # 找出面积最大的区域
        _, contours, _ = cv.findContours(mask3, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        maxArea = 0
        maxIndex = 0
        for i, c in enumerate(contours):
            area = cv.contourArea(c)
            if area > maxArea:
                maxArea = area
                maxIndex = i
        # 绘制
        cv.drawContours(cv_image, contours, maxIndex, (255, 255, 0), 2)

        # 获取外切矩形
        x, y, w, h = cv.boundingRect(contours[maxIndex])
        cv.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        # 获取中心像素点
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)
        cv.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

        cv.imshow("Image", cv_image)

        # 简单的打印反馈数据，之后补充运动控制
        twist = Twist()
        if center_x < screen_center - offset:
            twist.linear.x = 0.0
            twist.linear.y = 0.2
            twist.angular.z = 0.2
            print "turn left"
        elif screen_center - offset <= center_x <= screen_center + offset:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0
            print "keep"
        elif center_x > screen_center + offset:
            twist.linear.x = 0.0
            twist.linear.y = -0.2
            twist.angular.z = -0.2
            print "turn right"
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            print "stop"

        if center_y < screen_center_h - offset_h:
            twist.linear.z = 0.2
            print "up up up"
        elif screen_center_h - offset_h <= center_y <= screen_center_h + offset_h:
            twist.linear.z = 0
            print "keep"
        elif center_y > screen_center_h + offset_h:
            twist.linear.z = -0.2
            print "down down down"
        else:
            twist.linear.z = 0
            print "stop"

        cv.waitKey(3)

        try:
            self.cmd_pub.publish(twist)
        except CvBridgeError as e:
            print e


if __name__ == '__main__':
    try:
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()

    except KeyboardInterrupt:
        print "Shutting down cv_bridge_test node."
        cv.destroyAllWindows()




