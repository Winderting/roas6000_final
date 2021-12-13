#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@time       :2021/11/30 10:40:21
@author     :Mingdong
'''

import os
import cv2

from cv_bridge import CvBridge
import rospy
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import random


class VisualServo():
    def __init__(self):
        # 防止运算量过大，关闭laser?
        # self.pub_laser_switch = rospy.Publisher('/vrep/laser_switch', Bool, queue_size=1)
        # self.pub_laser_switch(True)

        rospy.Subscriber('/vrep/image', Image, self.callback, queue_size=1)
        self.pub_cmd = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=1)
        self._twist = Twist()
        self._twist.linear.x = 0.0
        self._twist.angular.z = 0.0


        self._bridge = CvBridge()

        self._width = 0
        self._height = 0
        self.low_range = np.array([16,100,50])
        self.up_range = np.array([30, 255, 255])


    def callback(self, img):
        rospy.loginfo(rospy.get_caller_id() + 'image receoved')
        camera_img = self._bridge.imgmsg_to_cv2(img, "bgr8")
        self._height = camera_img.shape[0]
        self._width = camera_img.shape[1]

        c_x, c_y, radius = self.img_find(camera_img)
        linear, angular = self.img2mov(c_x, c_y, radius)
        self._set_velocity(linear, angular)
        self._publish()


    def img_find(self, img):
        center_x = 0.0
        center_y = 0.0
        h, w = 0.0, 0.0
        radius = 0.0
        # area = 0.0

        # hough确定中心
        hough_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hough_image, self.low_range, self.up_range)

        # 形态学滤波
        kernel_open=np.ones((8,8))
        kernel_close=np.ones((25,25))
        # 腐蚀+膨胀
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel_open)
        # 膨胀加腐蚀
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel_close)
        # 镜像
        mask = cv2.flip(mask,1)
        img = cv2.flip(img,1)


        # hough circle
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=10, maxRadius=200)
        if circles is not None:
            center_x, center_y, radius = circles[0][0]
            h = 2*radius
            w = 2*radius
            # area = h * w
            # center = (x, y)
            # cv2.circle(img, center, radius, (0,255, 0), 2)
            # cv2.circle(mask, (center_x,center_y), radius, (255,0, 0), 2)
            cv2.drawMarker(img,(center_x,center_y),(255,0,0), markerType=0,markerSize=20,thickness=2)
            
        cv2.imshow("Mask Camera",mask)
        cv2.imshow("Camera",img)

        cv2.waitKey(1)

        return center_x, center_y, radius


    def img2mov(self, x, y, radius):
        # x,y are position of the circle center
        linear = 0.0
        angular = 0.0
        center_memory  = 0

        area_ratio = (radius*radius)/(self._width*self._height)
        if area_ratio:
            if x-self._width//2>=0:
                center_memory = 1
            else:
                center_memory = -1

            # 有visual目标
            linear = 0.4
            angular = 0.75*float(x-self._width//2)/(self._width//2)
            angular = -angular

            # 距离远则shrink调整角度
            if area_ratio<0.05:
                linear = 0.6
                angular = 0.2*angular/abs(angular)
                
            # 距离近,或需要倒退
            if area_ratio>0.1:
                linear = -2
                angular = 0.2*angular

        else:
            # # 失去目标随机探索
            # if random.random() < 0.5:
            #     angular = 2
            #     linear = 0.1
            # else:
            #     angular = -2
            #     linear = 0.1

            if center_memory>0:
                angular = 2
                linear = 0.1
            else:
                angular = -2
                linear = 0.1


            
        
        return linear, angular


    def _set_velocity(self,linear, angular):
        rospy.loginfo("set velocity")
        self._linear = linear
        self._angular = angular

    
    def _publish(self):
        rospy.loginfo("twist publish")
        self._twist.linear.x = self._linear; 
        self._twist.linear.y = 0; 
        self._twist.linear.z = 0
        self._twist.angular.x = 0; 
        self._twist.angular.y = 0; 
        self._twist.angular.z = self._angular
        self.pub_cmd.publish(self._twist)


if __name__ == '__main__':
    rospy.init_node('visual_servo')
    vs = VisualServo()
    rospy.spin()


    # # hough circle
    # # HSV色彩关注
    # # https://blog.csdn.net/u010429424/article/details/72989870

    # # Step1. 转换为HSV
    # # debug用vscode支持的路径
    # png_path = os.path.join(os.getcwd(), 'src/visual_servoing/ball2.png')
    
    # # png_path = './visual_servoing/ball2.png'
    # img = cv2.imread(png_path)
    # hough_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # # Step2. 用颜色分割图像
    # low_range = np.array([16,100,50])
    # high_range = np.array([30, 255, 255])
    # mask = cv2.inRange(hough_image, low_range, high_range)

    # # # Step3. 形态学运算，膨胀
    # # 形态学滤波
    # kernelOpen=np.ones((8,8))
    # kernelClose=np.ones((25,25))
    # # 腐蚀+膨胀
    # mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernelOpen)
    # # 膨胀加腐蚀
    # mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernelClose)

    # # # Step4. Hough Circle
    # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 100, param1=15, param2=7, minRadius=10, maxRadius=200)

    # # # Step5. 绘制
    # if circles is not None:
    #     x, y, radius = circles[0][0]
    #     center = (x, y)
    #     cv2.circle(img, center, radius, (0,255, 0), 2)
    

    # cv2.imshow("Mask Camera",img)
    # cv2.waitKey(0)
