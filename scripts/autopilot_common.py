#!/usr/bin/env python
# encoding: utf-8
import threading

import yaml
import time
import math
import rospy
import rospkg
import cv2 as cv
from numpy import *
import numpy as np
from cv_bridge import CvBridge
from yahboomcar_msgs.msg import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import LaserScan, Image

RAD2DEG = 180 / pi

def linear(pos1, pos2):
    '''
    线性公式：y=ax+b
    求a，b
    '''
    x1, y1 = pos1
    x2, y2 = pos2
    a = (y2 - y1) / (x2 - x1)
    b = y1 - (y2 - y1) * x1 / (x2 - x1)
    return a, b

def cacl_oblique_angle(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    x = (x1 - x2)
    y = (y1 - y2)
    angle = arctan(x / y) * RAD2DEG
    return angle

def ManyImgs(scale, imgarray):
    rows = len(imgarray)  # 元组或者列表的长度  Length of tuple or list
    cols = len(imgarray[0])  # 如果imgarray是列表，返回列表里第一幅图像的通道数，如果是元组，返回元组里包含的第一个列表的长度
    # If imgarray is a list, return the number of channels of the first image in the list, if it is a tuple, return the length of the first list contained in the tuple

    # print("rows=", rows, "cols=", cols)
    # 判断imgarray[0]的类型是否是list
    # 是list，表明imgarray是一个元组，需要垂直显示
    # Determine whether the type of imgarray[0] is list
    # It is a list, indicating that imgarray is a tuple and needs to be displayed verticallyIt is a list, indicating that imgarray is a tuple and needs to be displayed vertically
    rowsAvailable = isinstance(imgarray[0], list)
    # 第一张图片的宽高
    # The width and height of the first picture
    width = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    # print("width=", width, "height=", height)
    # 如果传入的是一个元组
    # If the incoming is a tuple
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                # 遍历元组，如果是第一幅图像，不做变换
                # Traverse the tuple, if it is the first image, do not transform
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                # 将其他矩阵变换为与第一幅图像相同大小，缩放比例为scale
                # Transform other matrices to the same size as the first image, and the zoom ratio is scale
                else:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (imgarray[0][0].shape[1], imgarray[0][0].shape[0]), None,
                                               scale, scale)
                # 如果图像是灰度图，将其转换成彩色显示
                # If the image is grayscale, convert it to color display
                if len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        # 创建一个空白画布，与第一张图片大小相同
        # Create a blank canvas, the same size as the first picture
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows  # 与第一张图片大小相同，与元组包含列表数相同的水平空白图像
        # The same size as the first picture, and the same number of horizontal blank images as the tuple contains the list
        for x in range(0, rows):
            # 将元组里第x个列表水平排列
            # Arrange the x-th list in the tuple horizontally
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)  # 将不同列表垂直拼接
        # Concatenate different lists vertically
    # 如果传入的是一个列表
    # If the incoming is a list
    else:
        # 变换操作，与前面相同
        # Transformation operation, same as before
        for x in range(0, rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(imgarray[x], (imgarray[0].shape[1], imgarray[0].shape[0]), None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        # 将列表水平排列
        # Arrange the list horizontally
        hor = np.hstack(imgarray)
        ver = hor
    return ver

class HSVYaml:
    def __init__(self):
        self.hsv_text = rospkg.RosPack().get_path("arm_autopilot") + "/scripts/HSV.yaml"

    def write_hsv(self, color, hsv):
        with open(self.hsv_text, 'r') as f:
            result = yaml.load_all(f.read(), Loader=yaml.FullLoader)
            for i in result:
                i[color] = hsv
                color_hsv = i
        with open(self.hsv_text, 'w') as f:
            yaml.safe_dump(color_hsv, f)

    def read_hsv(self,name):
        with open(self.hsv_text, 'r') as f:
            result = yaml.load_all(f.read(), Loader=yaml.FullLoader)
            for i in result: color_hsv = i
        return color_hsv[name]


class ROSCtrl:
    def __init__(self):
        self.RobotRun_status = True
        self.Buzzer_state = False
        self.Joy_active = False
        self.LaserAngle = 30
        self.ResponseDist = 0.55
        self.warning = 1
        self.joy_action = 0
        self.img_flip = rospy.get_param("~img_flip", False)
        self.VideoSwitch = rospy.get_param("~VideoSwitch", False)
        self.VideoSwitch = rospy.get_param("~VideoSwitch", False)
        self.sub_JoyState = rospy.Subscriber('/JoyState', Bool, self.JoyStateCallback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        self.pub_CmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_buzzer = rospy.Publisher('/Buzzer', Bool, queue_size=1)
        self.pub_rgb = rospy.Publisher("/linefollw/rgb", Image, queue_size=1)
        self.pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
        self.sub_RGBLight = rospy.Subscriber("RGBLight", Int32, self.RGBLightcallback, queue_size=100)

    def RGBLightcallback(self, msg):
        if not isinstance(msg, Int32): return
        threading.Thread(target=self.joy_action_update,).start()

    def joy_action_update(self):
        self.joy_action += 1
        time.sleep(0.5)
        self.joy_action = 0

    def pubArm(self, joints, id=10, angle=90, run_time=500):
        armjoint = ArmJoint()
        armjoint.run_time = run_time
        if len(joints) != 0: armjoint.joints = joints
        else:
            armjoint.id = id
            armjoint.angle = angle
        self.pub_Arm.publish(armjoint)

    def pubVel(self, x, y, z=0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = z
        self.pub_CmdVel.publish(twist)
        self.RobotRun_status = False

    def pubImg(self, rgb_img):
        self.bridge = CvBridge()
        self.pub_rgb.publish(self.bridge.cv2_to_imgmsg(rgb_img, "bgr8"))

    def pubBuzzer(self, status):
        self.pub_buzzer.publish(status)
        self.Buzzer_state = False

    def JoyStateCallback(self, msg):
        if not isinstance(msg, Bool): return
        self.Joy_active = msg.data
        self.pubVel(0, 0)

    def cancel(self):
        self.pub_CmdVel.publish(Twist())
        self.pub_rgb.unregister()
        self.pub_buzzer.unregister()
        self.pub_CmdVel.unregister()
        self.sub_scan.unregister()
        self.sub_JoyState.unregister()

    def registerScan(self, scan_data):
        self.warning = 1
        if not isinstance(scan_data, LaserScan): return
        if self.Joy_active == True: return
        # 记录激光扫描并发布最近物体的位置（或指向某点）
        ranges = np.array(scan_data.ranges)
        # 按距离排序以检查从较近的点到较远的点是否是真实的东西
        # if we already have a last scan to compare to:
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
            # 通过清除不需要的扇区的数据来保留有效的数据
            if abs(angle) > (180 - self.LaserAngle):
                if ranges[i] < self.ResponseDist: self.warning += 1


class color_follow:
    def __init__(self):
        self.frame = None
        self.binary = ()
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.msg_box = {}
        self.msg_circle = {}
        self.target_color_name = 'red'
        self.color_hsv_list = {
            "red": ((176, 78, 65), (180, 253, 255)),
            "green": ((46, 37, 71), (99, 234, 255)),
            "blue": ((104, 137, 108), (127, 253, 255)),
            "yellow": ((30, 24, 139), (33, 158, 255)),
        }
        # self.color_hsv_list = {
        #     "red": ((0, 43, 46), (10, 253, 255)),
        #     "green": ((35, 43, 46), (77, 253, 255)),
        #     "blue": ((100, 43, 46), (124, 253, 255)),
        #     "yellow": ((26, 43, 46), (34, 253, 255)),
        # }
        self.hsv_yaml = HSVYaml()

    def line_follow(self, rgb_img, color_name, hsv_msg):
        height, width = rgb_img.shape[:2]
        img = rgb_img.copy()
        if color_name==self.target_color_name: img[0:int(height / 2), 0:width] = 0
        # 将图像转换为HSV。
        # Convert the image to HSV.
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # 根据特定颜色范围创建mask
        # Create a mask based on a specific color range
        mask = cv.inRange(hsv_img, lower, upper)
        color_mask = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
        # 将图像转为灰度图
        # Convert the image to grayscale
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        # Get structure elements of different shapes
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        # Morphological closed operation
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
        # Image binarization operation
        ret, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标)
        # Get the set of contour points (coordinates)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        box = [];circle = ()
        if len(contours) != 0:
            areas = []
            for c in range(len(contours)): areas.append(cv.contourArea(contours[c]))
            # print "max_areas",max(areas)
            if max(areas) > 800:
                max_id = areas.index(max(areas))
                max_rect = cv.minAreaRect(contours[max_id])
                max_box = cv.boxPoints(max_rect)
                box = np.int0(max_box)
                (color_x, color_y), color_radius = cv.minEnclosingCircle(box)
                # 将检测到的颜色用原形线圈标记出来
                # Mark the detected color with the original shape coil
                circle = int(color_x), int(color_y)
        if color_name==self.target_color_name: self.binary = binary
        self.msg_box[color_name] = box
        self.msg_circle[color_name] = circle

    def add_box(self, color_name):
        if len(self.msg_box[color_name]) != 0:
            cv.drawContours(self.frame, [self.msg_box[color_name]], 0, (255, 0, 0), 2)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(self.msg_box[color_name])
            Center_x, Center_y, Center_r = int(color_x), int(color_y), int(color_radius)
            cv.circle(self.frame, (Center_x, Center_y), 5, (255, 0, 255), -1)
            cv.putText(self.frame, color_name, (Center_x, Center_y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 200), 1)

    def Roi_hsv(self, img, Roi):
        '''
        Get the range of HSV in a certain area获取某一区域的HSV的范围
        :param img: 彩色图
        :param Roi:  (x_min, y_min, x_max, y_max)
        Roi=(290,280,350,340)
        :return: The range of images and HSV such as:(0,0,90)(177,40,150) 图像和HSV的范围 例如：(0,0,90)(177,40,150)
        '''
        H = [];S = [];V = []
        # img = cv.resize(img, (640, 480), )
        # 将彩色图转成HSV
        # Convert color image to HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # 画矩形框
        # Draw a rectangular frame
        # cv.rectangle(img, (Roi[0], Roi[1]), (Roi[2], Roi[3]), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中
        # Take out the H, S, V values of each row and each column in turn and put them into the container
        for i in range(Roi[0], Roi[2]):
            for j in range(Roi[1], Roi[3]):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # 分别计算出H,S,V的最大最小
        # Calculate the maximum and minimum of H, S, and V respectively
        H_min = min(H); H_max = max(H)
        S_min = min(S); S_max = 253
        V_min = min(V); V_max = 255
        # HSV范围调整
        # HSV range adjustment
        if H_max + 5 > 180: H_max = 180
        else: H_max += 5
        if H_min - 5 < 0: H_min = 0
        else: H_min -= 5
        if S_min - 10 < 0: S_min = 0
        else: S_min -= 10
        if V_min - 10 < 0: V_min = 0
        else: V_min -= 10
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


class simplePID:
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        # rospy.loginfo('P:{}, I:{}, D:{}'.format(P, I, D))
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update(self, current_value):
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D
