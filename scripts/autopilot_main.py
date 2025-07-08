#!/usr/bin/env python
# encoding: utf-8
import os
import threading
from time import sleep
from autopilot_common import *
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from arm_autopilot.cfg import AutoPilotPIDConfig


class LineDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        rospy.init_node("LineDetect", anonymous=False)
        self.ros_ctrl = ROSCtrl()
        self.color = color_follow()
        self.hsv_yaml = HSVYaml()
        self.dyn_update = False
        self.Calibration = False
        self.select_flags = False
        self.gripper_state = False
        self.location_state = False
        self.Track_state = 'identify'
        self.windows_name = 'frame'
        
        # Starea initiala a robotului, cu culorile de traseu actualizate
        self.path_colors = ['red', 'yellow']
        self.current_target_color = self.path_colors[0] # Culoarea pe care o urmareste acum
        self.intended_target_color = self.path_colors[0] # Culoarea pe care o cauta
        self.color.target_color_name = self.path_colors[0] # Culoarea pentru calibrare

        self.color_name_list = ['red', 'green', 'blue', 'yellow']
        self.hsv_value = ()
        self.color_cfg_src = self.index = self.cols = self.rows = 0
        self.Mouse_XY = (0, 0)
        self.Roi_init = ()
        Server(AutoPilotPIDConfig, self.dyn_cfg_callback)
        self.dyn_client = Client("LineDetect", timeout=60)
        self.scale = 1000.0
        self.FollowLinePID = (30.0, 0.0, 60.0)
        self.linear = 0.10
        self.PID_init()
        self.joints_init = [90, 120, 0, 0, 90, 30]
        for i in range(4):
            self.color.color_hsv_list[self.color_name_list[i]] = self.hsv_yaml.read_hsv(self.color_name_list[i])
        cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
        cv.setMouseCallback(self.windows_name, self.onMouse, 0)
        self.ros_ctrl.pubArm(self.joints_init)
        self.is_searching = False
        self.last_known_error = 0

    def process(self, rgb_img, action):
        self.color.msg_circle = {}
        self.color.msg_box = {}

        if action == 32 or self.ros_ctrl.joy_action == 2:
            self.Track_state = 'tracking'
            self.Calibration = False
            self.dyn_update = True
            self.ros_ctrl.pubArm(self.joints_init)
        elif action == ord('r') or action == ord('R'):
            self.Reset()
        elif action == ord('q') or action == ord('Q'):
            self.cancel()
        elif action == ord('c') or action == ord('C'):
            self.Calibration = not self.Calibration
            rospy.loginfo("Mod Calibrare: {}".format(self.Calibration))
            self.dyn_update = True
        elif action == ord('i') or action == ord('I'):
            self.Track_state = "identify"
            self.Calibration = False
            self.dyn_update = True
        
        elif action == ord('f') or action == ord('F'):
            if self.Calibration:
                current_index = self.color_name_list.index(self.color.target_color_name)
                next_index = (current_index + 1) % len(self.color_name_list)
                self.color.target_color_name = self.color_name_list[next_index]
                self.hsv_value = self.hsv_yaml.read_hsv(self.color.target_color_name)
                self.dyn_update = True
                rospy.loginfo("Selectat pentru calibrare: {}".format(self.color.target_color_name))
        
        elif action == ord('s') or action == ord('S'):
            if self.Track_state == 'tracking':
                if self.intended_target_color == self.path_colors[0]:
                    self.intended_target_color = self.path_colors[1]
                else:
                    self.intended_target_color = self.path_colors[0]
                rospy.loginfo("Intentie noua: Caut linia [{}].".format(self.intended_target_color))

        if self.Track_state == 'init':
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    rgb_img, self.hsv_value = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.color.color_hsv_list[self.color.target_color_name] = self.hsv_value
                    self.hsv_yaml.write_hsv(self.color.target_color_name, self.hsv_value)
                    self.dyn_update = True
                else: self.Track_state = 'init'

        if self.Track_state != 'init' and len(self.hsv_value) != 0:
            if self.Calibration:
                self.color.line_follow(rgb_img, self.color.target_color_name, self.hsv_value)
            else:
                threads = []
                for color_name in self.path_colors:
                    thread = threading.Thread(target=self.color.line_follow,
                                         args=(rgb_img, color_name,
                                               self.color.color_hsv_list[color_name],))
                    threads.append(thread)
                    thread.start()
                for thread in threads:
                    thread.join()

        if self.Track_state == 'tracking' and not self.ros_ctrl.Joy_active and not self.gripper_state:
            is_intended_detected = self.intended_target_color in self.color.msg_circle and len(self.color.msg_circle.get(self.intended_target_color, ())) != 0

            if is_intended_detected:
                if self.current_target_color != self.intended_target_color:
                    rospy.loginfo("Am gasit linia [{}], am comutat!".format(self.intended_target_color))
                    self.current_target_color = self.intended_target_color
                    self.color.target_color_name = self.current_target_color
                    self.dyn_update = True
                self.execute(self.color.msg_circle[self.current_target_color])
            else:
                is_current_detected = self.current_target_color in self.color.msg_circle and len(self.color.msg_circle.get(self.current_target_color, ())) != 0
                if is_current_detected:
                    if self.is_searching:
                       rospy.loginfo("Linia [{}] a fost regasita!".format(self.current_target_color))
                       self.is_searching = False
                    self.execute(self.color.msg_circle[self.current_target_color])
                else:
                    self.start_search_routine()
        else:
            if self.ros_ctrl.RobotRun_status and not self.gripper_state:
                self.ros_ctrl.pubVel(0, 0)

        if self.dyn_update:
            self.dyn_cfg_update()
        return self.color.binary

    def Wrecker(self, point_x, point_y):
        pass

    def robot_location(self, point_x, point_y):
        pass

    def arm_gripper(self, joints):
        pass

    def execute(self, circle):
        if len(circle) == 0:
            self.ros_ctrl.pubVel(0, 0)
        else:
            if self.ros_ctrl.warning > 10:
                self.ros_ctrl.pubVel(0, 0)
            else:
                self.last_known_error = circle[0] - 320
                [z_Pid, _] = self.PID_controller.update([(self.last_known_error) / 16, 0])
                z = -z_Pid if self.ros_ctrl.img_flip else z_Pid
                self.ros_ctrl.pubVel(self.linear, 0, z=z)
                self.ros_ctrl.RobotRun_status = True

    def start_search_routine(self):
        if not self.is_searching:
            rospy.loginfo("Linia [{}] pierduta! Incep cautarea...".format(self.current_target_color))
            self.is_searching = True
        search_angular_speed = -0.6 if self.last_known_error > 0 else 0.6
        self.ros_ctrl.pubVel(0, 0, z=search_angular_speed)
        self.ros_ctrl.RobotRun_status = True

    def dyn_cfg_update(self):
        hsv = self.color.color_hsv_list[self.color.target_color_name]
        try: color_index = self.color_name_list.index(self.color.target_color_name)
        except ValueError: color_index = 0
        params = {
            'Calibration': self.Calibration, 'Color': color_index,
            'Hmin': hsv[0][0], 'Hmax': hsv[1][0], 'Smin': hsv[0][1], 'Smax': hsv[1][1],
            'Vmin': hsv[0][2], 'Vmax': hsv[1][2]}
        self.dyn_client.update_configuration(params)
        self.dyn_update = False

    def dyn_cfg_callback(self, config, level):
        self.scale = config['scale']
        self.linear = config['linear']
        self.ros_ctrl.LaserAngle = config['LaserAngle']
        self.ros_ctrl.ResponseDist = config['ResponseDist']
        self.FollowLinePID = (config['Kp'], config['Ki'], config['Kd'])
        if self.Calibration:
            if self.color_name_list[config["Color"]] != self.color.target_color_name:
                 self.color.target_color_name = self.color_name_list[config["Color"]]
                 self.hsv_value = self.hsv_yaml.read_hsv(self.color.target_color_name)
                 self.dyn_update = True
            else:
                 self.hsv_value = ((config['Hmin'], config['Smin'], config['Vmin']), (config['Hmax'], config['Smax'], config['Vmax']))
                 self.hsv_yaml.write_hsv(self.color.target_color_name, self.hsv_value)
                 self.color.color_hsv_list[self.color.target_color_name] = self.hsv_value
        self.PID_init()
        return config

    def putText_img(self, frame):
        if self.Calibration:
            calib_text = "Calibrez: " + self.color.target_color_name.upper()
            cv.putText(frame, calib_text, (350, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            txt_current = "Urmaresc: " + self.current_target_color
            txt_intended = "Caut: " + self.intended_target_color
            cv.putText(frame, txt_current, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv.putText(frame, txt_intended, (30, 80), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        self.ros_ctrl.pubImg(frame)
        return frame

    def onMouse(self, event, x, y, flags, param):
        if x > 640 or y > 480: return
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Calibration = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def Reset(self):
        self.current_target_color = self.path_colors[0]
        self.intended_target_color = self.path_colors[0]
        self.color.target_color_name = self.path_colors[0]
        self.PID_init()
        self.Track_state = 'identify'
        self.is_searching = False
        self.gripper_state = False
        self.ros_ctrl.Joy_active = False
        self.ros_ctrl.pubVel(0, 0)
        rospy.loginfo("Reset succes!!!")

    def PID_init(self):
        self.PID_controller = simplePID([0, 0], [self.FollowLinePID[0] / self.scale, self.FollowLinePID[0] / self.scale], [self.FollowLinePID[1] / self.scale, self.FollowLinePID[1] / self.scale], [self.FollowLinePID[2] / self.scale, self.FollowLinePID[2] / self.scale])

    def cancel(self):
        self.Reset()
        self.ros_ctrl.cancel()
        print("Shutting down this node.")


if __name__ == '__main__':
    line_detect = LineDetect()
    capture = cv.VideoCapture('/dev/camera_usb')
    cv_edition = cv.__version__
    if cv_edition[0] == '3':
        capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'XVID'))
    else:
        capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    while capture.isOpened():
        start = time.time()
        ret, frame = capture.read()
        if not ret: break
        action = cv.waitKey(10) & 0xFF
        if line_detect.ros_ctrl.img_flip: frame = cv.flip(frame, 1)
        line_detect.color.frame = frame
        binary = line_detect.process(frame, action)
        end = time.time()
        fps = 1 / (end - start)
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        frame = line_detect.putText_img(frame)
        if len(binary) != 0:
            cv.imshow(line_detect.windows_name, ManyImgs(1, ([frame, binary])))
        else:
            cv.imshow(line_detect.windows_name, frame)
        if action == ord('q') or action == 113: break
    capture.release()
    cv.destroyAllWindows()
