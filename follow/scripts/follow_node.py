#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2020/11/6 下午3:11
# @Author : Chenan_Wang
# @File : follow_node.py
# @Software: CLion

import rospy
import cv2 as cv
from geometry_msgs.msg import Twist


def shutdown():
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = 0
    cmd_vel_Publisher.publish(twist)
    print "stop"


if __name__ == '__main__':
    rospy.init_node("yellow_follow")

    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)

    cmd_vel_Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # 定义结构元素
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (3, 3))
    # print kernel

    capture = cv.VideoCapture(0)
    print capture.isOpened()
    ok, frame = capture.read()
    lower_b = (65, 43, 46)
    upper_b = (110, 255, 255)

    height, width = frame.shape[0:2]
    screen_center = width / 2
    offset = 50

    while not rospy.is_shutdown():
        # 将图像转成HSV颜色空间
        hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        # 基于颜色的物体提取
        mask = cv.inRange(hsv_frame, lower_b, upper_b)
        mask2 = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask3 = cv.morphologyEx(mask2, cv.MORPH_CLOSE, kernel)

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
        cv.drawContours(frame, contours, maxIndex, (255, 255, 0), 2)
        # 获取外切矩形
        x, y, w, h = cv.boundingRect(contours[maxIndex])
        cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        # 获取中心像素点
        center_x = int(x + w / 2)
        center_y = int(y + h / 2)
        cv.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # 简单的打印反馈数据，之后补充运动控制
        twist = Twist()
        if center_x < screen_center - offset:
            twist.linear.x = 0.05
            twist.angular.z = 0.2
            print "turn left"
        elif screen_center - offset <= center_x <= screen_center + offset:
            twist.linear.x = 0.1
            twist.angular.z = 0
            print "keep"
        elif center_x > screen_center + offset:
            twist.linear.x = 0.05
            twist.angular.z = -0.2
            print "turn right"
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            print "stop"

        # fachuqu
        cmd_vel_Publisher.publish(twist)

        cv.imshow("mask4", mask3)
        cv.imshow("frame", frame)
        cv.waitKey(1)
        rate.sleep()
        ok, frame = capture.read()