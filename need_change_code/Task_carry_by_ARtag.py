#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import tty
import termios
import numpy as np
import scipy.linalg as linalg
if sys.version>'3':
    import configparser as ConfigParser
else:
    import ConfigParser

import rospy
import rospkg

import cv2
import numpy as np
from cv_bridge import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *

from mediumsize_msgs.srv import SetAction#定义消息

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli#take
import vision.imageProcessing as imgPrcs
import algorithm.pidAlgorithm as pidAlg
from lejulib import *
#抓取、放置
# 添加脚本路径以便导入 client_action 与 frames
try:
    import client_action
except Exception:
    client_action = None
try:
    from frames import RobanFrames
except Exception:
    RobanFrames = None

CFG_FILE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "./Task_carry_artag.conf"))
NODE_NAME = 'stair_convert'
CHIN_POS_TOPIC = "/chin/visualization_marker_chin"
HEAD_POS_TOPIC = "/head/visualization_marker_head"

CONTROL_ID = 2

GOAL_POS_CHIN = [0.130, 0.220, 0.0]         # 下巴摄像头目标位置
GOAL_POS_HEAD = [0.130, 0.220, 0.0]         # 头部摄像头目标位置

WAIT_FOR_ARTAG_DATA_STABLE_TIME_OUT = 3     # 等待ar_tag数据稳定的最长时间
ARTAG_TOPIC_RATE = 8
WAIT_ARTAG_DATA_STABLIZED = 2
WAIT_FOR_ARTAG_DATA_REFRESH = 0.5

def terminate(data):
    rospy.loginfo(data.data)
    rospy.signal_shutdown("kill")

class Action(object):
    '''
    robot action
    '''

    def __init__(self, name, ctl_id, disable_signals=False):
        rospy.init_node(name, anonymous=True, disable_signals=disable_signals)
        time.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)

        self.bodyhub = bodycli.BodyhubClient(ctl_id)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def start(self):
        print( 'action start')


class TaskCarryConvert(Action):
    def __init__(self, node_name=NODE_NAME, disable_signals=False):
        super(TaskCarryConvert, self).__init__(node_name, CONTROL_ID, disable_signals)
        rospy.Subscriber('terminate_current_process', String, terminate)       
          
        self.x = []
        self.y = []
        self.angle = []
        self.datalen = int(ARTAG_TOPIC_RATE * WAIT_ARTAG_DATA_STABLIZED)
        self.limit = [0.01, 0.01, 1.0]

        # 订阅ar_tag位置话题
        rospy.Subscriber(CHIN_POS_TOPIC, Marker, self.chin_marker_callback, queue_size=1)
        rospy.Subscriber(HEAD_POS_TOPIC, Marker, self.head_marker_callback, queue_size=1) 

        self.goal_pos_chin = GOAL_POS_CHIN      # 下巴摄像头目标位置
        self.goal_pos_head = GOAL_POS_HEAD      # 头部摄像头目标位置

        self.pid_gain = [0.7, 0.8, 0.9]
        self.err_threshold = [0.02, 0.02, 2.0]      # 误差阈值设定
        self.cf = ConfigParser.ConfigParser()
        print( '加载配置文件: {}'.format(CFG_FILE_PATH))
        self.load_cfg()
        print(' 目标位置: {}'.format(self.goal_pos_chin))

        self.tag_id = -1        # 记录当前识别到的ar_tag id
        self.take_tag_id = 2    # 取货点的tag id
        self.place_tag_id = -1   # 放货点的tag id

        self.real_pos_chin = [0.0, 0.0, 0.0]        # 下巴摄像头识别到的位置
        self.real_pos_head = [0.0, 0.0, 0.0]        # 头部摄像头识别到的位置

        self.pos_valid = False
        self.pid_x = pidAlg.PositionPID(p=self.pid_gain[0])
        self.pid_y = pidAlg.PositionPID(p=self.pid_gain[1])
        self.pid_a = pidAlg.PositionPID(p=self.pid_gain[2])

    def rotate_mat(self, axis, radian):
        rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
        return rot_matrix

    def quart_to_rpy(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]

    def append_artag_data(self, x, y, angle):
        if len(self.x) > self.datalen:
            self.x.pop(0)
        self.x.append(x)
        if len(self.y) > self.datalen:
            self.y.pop(0)
        self.y.append(y)
        if len(self.angle) > self.datalen:
            self.angle.pop(0)
        self.angle.append(angle)

    def chin_marker_callback(self, msg):
        if math.isnan(msg.pose.position.x): return
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        cam_rot = self.rotate_mat([0, 1, 0], 10 * math.pi / 180.0) # 绕y轴旋转  
        pos_in_torso = np.dot(cam_rot, np.array([[-msg.pose.position.y], [-msg.pose.position.x], [msg.pose.position.z]])).tolist()
        self.real_pos_chin[0] = pos_in_torso[0][0]
        self.real_pos_chin[1] = pos_in_torso[1][0]
        self.real_pos_chin[2] = -rpy[2] * 180.0 / math.pi
        self.append_artag_data(self.real_pos_chin[0], self.real_pos_chin[1], self.real_pos_chin[2])
        self.pos_valid = True

        self.tag_id = msg.id
        # if(self.tag_id != -1):
        #     print('当前识别到的tag id: ', self.tag_id)
        # else:
        #     print('当前未找到tag码')
   


    def head_marker_callback(self, msg):
        if math.isnan(msg.pose.position.x): return
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        cam_rot = self.rotate_mat([0, 1, 0], 10 * math.pi / 180.0) # 绕y轴旋转  
        pos_in_torso = np.dot(cam_rot, np.array([[-msg.pose.position.y], [-msg.pose.position.x], [msg.pose.position.z]])).tolist()
        self.real_pos_head[0] = pos_in_torso[0][0]
        self.real_pos_head[1] = pos_in_torso[1][0]
        self.real_pos_head[2] = -rpy[2] * 180.0 / math.pi
        self.append_artag_data(self.real_pos_head[0], self.real_pos_head[1], self.real_pos_head[2])
        self.pos_valid = True

        self.tag_id = msg.id
        # if(self.tag_id != -1):
        #     print('当前识别到的tag id: ', self.tag_id)
        # else:
        #     print('当前未找到tag码')
        
        
    @property
    def is_stablized(self):
        if len(self.x) < self.datalen:
            return False
        if max(self.x) - min(self.x) < self.limit[0] and max(self.y) - min(self.y) < self.limit[1] and max(self.angle) - min(self.angle) < self.limit[2]:
            return True
        return False

    @property
    def is_artag_data_refresh(self):
        self.pos_valid = False
        stime = time.time()
        while time.time() - stime < WAIT_FOR_ARTAG_DATA_REFRESH:
            if self.pos_valid: return True
        return False

    def goto_rot(self, goal_ang):
        self.bodyhub_walk()
        corrtLength = 0  # 修正距离，旋转过度时出现找不到ar_tag时，通过该角度回转
        while not rospy.is_shutdown():
            stime = time.time()
            while time.time() - stime < WAIT_FOR_ARTAG_DATA_STABLE_TIME_OUT:
                if self.is_stablized: break
                rospy.sleep(0.2)
            if self.is_artag_data_refresh:
                aError = self.real_pos_chin[2] - goal_ang
                print('angel Error:',aError)
                if (abs(aError) < self.err_threshold[2]):
                    print('到达目标角度，当前位置:{} ,err:{}'.format(self.real_pos_chin, aError))
                    break
                aLength = self.pid_a.run(aError)
                corrtLength = aLength
                self.bodyhub.walking_the_distance(0.0, 0.0, aLength)
                # self.bodyhub.wait_walking_done()  # wait_walking_done中有pause状态， 会导致请求阻塞，不用这个
            else:
                rospy.logwarn('ar tag no found!')
                # 通过-corrtLength回到之前有tag的位置
                self.bodyhub.walking_the_distance(0.0, 0.0, -corrtLength)
                # self.bodyhub.wait_walking_done()

    def goto_pose(self, goal_pos, chin_or_hand):
        # self.err_threshold = [0.01, 0.02, 1.0]   # 误差阈值设定
        self.bodyhub_walk()
        correctLength = [0.0, 0.0, 0.0]  # 修正距离，找不到ar_tag时，回正用

        while not rospy.is_shutdown():
            stime = time.time()
            while time.time() - stime < WAIT_FOR_ARTAG_DATA_STABLE_TIME_OUT:
                if self.is_stablized: break
                rospy.sleep(0.2)
            
            if self.is_artag_data_refresh:
                if chin_or_hand == "chin":       # chin下巴摄像头  head头部摄像头
                    xError = self.real_pos_chin[0] - goal_pos[0]
                    yError = self.real_pos_chin[1] - goal_pos[1]
                    aError = self.real_pos_chin[2] - goal_pos[2]

                    print('【chin】当前位置:',self.real_pos_chin)
                    print('【chin】目标位置:',goal_pos)
                    print('位置误差',xError, yError, aError)
                    print('误差阈值：',self.err_threshold)


                elif chin_or_hand == "head":
                    xError = self.real_pos_head[0] - goal_pos[0]
                    yError = self.real_pos_head[1] - goal_pos[1]
                    aError = self.real_pos_head[2] - goal_pos[2]

                    print('【head】当前位置:',self.real_pos_head)
                    print('【head】目标位置:',goal_pos)
                    print('位置误差',xError, yError, aError)
                    print('误差阈值：',self.err_threshold)
                

                if (abs(xError) < self.err_threshold[0]) and (abs(yError) < self.err_threshold[1]) and (abs(aError) < self.err_threshold[2]):
                    print( '到达目标位置，当前位置:{} ,err:{} {} {}'.format(self.real_pos_head, xError, yError, aError))
                    break
                # 需要移动的距离
                xLength = self.pid_x.run(xError)
                yLength = self.pid_y.run(yError)
                aLength = self.pid_a.run(aError)
                correctLength[0] = -xLength
                correctLength[1] = -yLength
                correctLength[2] = -aLength
                self.bodyhub.walking_the_distance(xLength, yLength, aLength)
                # self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('ar tag no found!')
                
                # self.bodyhub.walking_the_distance(correctLength[0], correctLength[1], correctLength[2])   # 通过-corrtLength回到之前有tag的位置
                self.bodyhub.walking_n_steps([0.04, 0.0, 0.0], 1)   # 通过向前走4cm，尝试重新找到tag码
                # self.bodyhub.wait_walking_done()
    
    def load_cfg(self):
        self.cf.read(CFG_FILE_PATH)
        # 加载下巴摄像头识别的目标位置
        setion = self.cf.items('target_pos_chin')
        self.goal_pos_chin = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]
        # 加载头部摄像头识别的目标位置
        setion = self.cf.items('target_pos_head')
        self.goal_pos_head = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]

        setion = self.cf.items('pid_gain')
        self.pid_gain = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]
        setion = self.cf.items('err_threshold')
        self.err_threshold = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]

    # debug模式校准下巴和头部摄像头识别到的tag码目标位置
    def set_target_pos(self, chin_or_head):
        self.bodyhub_walk()
        print( '将机器人放到目标位置, 按下回车键校准')
        while not rospy.is_shutdown():
            if input('input:') == 'q':
                break
            if self.pos_valid and chin_or_head == 'chin':
                print('下巴摄像头位置校准')
                self.cf.set('target_pos_chin', 'x', str(self.real_pos_chin[0]))
                self.cf.set('target_pos_chin', 'y', str(self.real_pos_chin[1]))
                self.cf.set('target_pos_chin', 'yaw', str(self.real_pos_chin[2]))
                self.cf.write(open(CFG_FILE_PATH, 'w'))
                self.load_cfg()
                
                setion = self.cf.items('reference_pos')
                ref_pos = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]
                print( '参考位置: ', ref_pos)
                print( '校准后位置: ', self.goal_pos_chin)
                print( '校准完成，程序退出')
                break
            elif self.pos_valid and chin_or_head == 'head':
                print('头部摄像头位置校准')
                self.cf.set('target_pos_head', 'x', str(self.real_pos_head[0]))
                self.cf.set('target_pos_head', 'y', str(self.real_pos_head[1]))
                self.cf.set('target_pos_head', 'yaw', str(self.real_pos_head[2]))
                self.cf.write(open(CFG_FILE_PATH, 'w'))
                self.load_cfg()
                
                setion = self.cf.items('reference_pos')
                ref_pos = [float(setion[0][1]), float(setion[1][1]), float(setion[2][1])]
                print( '参考位置: ', ref_pos)
                print( '校准后位置: ', self.goal_pos_head)
                print( '校准完成，程序退出')
                break
            else:
                print( '没有识别到ar_tag码，请移动机器人或检查摄像头！')
                print( '按下回车键继续，按q键退出')
        rospy.signal_shutdown('exit')



    def take(self):
        """抓取动作：优先调用现有的高层接口（如果存在），否则尽量使用 bodyhub 的基础接口等待完成。"""
        # 设置手臂为动作模式（0 表示动作，1 表示步态）
        try:
            rospy.wait_for_service('MediumSize/BodyHub/armMode', timeout=5)
            svc = rospy.ServiceProxy('MediumSize/BodyHub/armMode', SetAction)
            svc(0, 'set arm mode')
        except Exception:
            rospy.logdebug('MediumSize/BodyHub/armMode service not available, continue')
        rospy.loginfo('take action: bodyhub_ready()')
        try:
            self.bodyhub_ready()
        except Exception:
            rospy.logwarn('bodyhub_ready failed or not available')
        # 执行动作帧：优先通过 client_action.custom_action -> send_custom_bezier
        if client_action is not None and RobanFrames is not None:
            try:
                client_action.custom_action([], RobanFrames.take_frames)
            except Exception as e:
                rospy.logwarn('failed to run custom_action for take: %s' % str(e))
        else:
            rospy.logwarn('client_action or RobanFrames not available; skipping action frames')
        try:
            self.bodyhub.wait_joint_done(True)
        except Exception:
            rospy.logdebug('bodyhub.wait_joint_done not available or failed')
        rospy.loginfo('take action: done')

    def place(self):
        """放置动作：与 take 对应的放置实现，优先高层接口，失败降级到基础接口。"""
        try:
            rospy.wait_for_service('MediumSize/BodyHub/armMode', timeout=5)
            svc = rospy.ServiceProxy('MediumSize/BodyHub/armMode', SetAction)
            svc(0, 'set arm mode')
        except Exception:
            rospy.logdebug('MediumSize/BodyHub/armMode service not available, continue')
        rospy.loginfo('place action: bodyhub_ready()')
        try:
            self.bodyhub_ready()
        except Exception:
            rospy.logwarn('bodyhub_ready failed or not available')
        if client_action is not None and RobanFrames is not None:
            try:
                client_action.custom_action([], RobanFrames.place_frames)
            except Exception as e:
                rospy.logwarn('failed to run custom_action for place: %s' % str(e))
        else:
            rospy.logwarn('client_action or RobanFrames not available; skipping action frames')
        try:
            self.bodyhub.wait_joint_done(True)
        except Exception:
            rospy.logdebug('bodyhub.wait_joint_done not available or failed')
        rospy.loginfo('place action: done')

    def start(self):
        # self.bodyhub_walk() 
    

        # ------------------取货部分测试------------------
        # slam.pubnode.set_arm_mode(1)        # 摆手
        slam.pubnode.set_arm_mode(0)        # 不摆手
        print('使用chin摄像头定位，移动到目标位置:', self.goal_pos_chin)
        self.goto_rot(0.0)      # 先对准角度方向
        # 移动到取货位置
        while self.tag_id != self.take_tag_id:
            self.goto_pose(self.goal_pos_chin,"chin")       # self.err_threshold 是误差精度
            # 走到一个tag码处，检查是不是取货点的tag码，如果不是则向前走4cm继续寻找
            if self.tag_id != self.take_tag_id:
                print('当前位置tag id:', self.tag_id, '不是取货点，继续前进寻找取货点tag id:', self.take_tag_id)
                self.bodyhub.walking_n_steps([0.02, 0.0, 0.0], 10)   
            else:
                print('找到取货点tag id:', self.tag_id)
                break
        print('到达取货点，tag id:', self.tag_id)

        # 取货
        self.take()
        self.bodyhub_walk()
        # ------------------取货部分测试------------------

        # 抱取货物，后退三步再旋转180度，前走
        self.bodyhub.walking_n_steps([-0.08, 0.0, 0.0], 3)
        self.bodyhub.walking_n_steps([0.0, 0.0, 10.0], 16)
        self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 2)

        # self.bodyhub.wait_walking_done()
        # ------------------放货部分测试------------------
        # slam.pubnode.set_arm_mode(0)        # 不摆手
        print('使用head摄像头定位，移动到目标位置:', self.goal_pos_head)
        # self.goto_rot(0.0)      # 先对准角度方向
        # 移动到放货位置
        self.goto_pose(self.goal_pos_head,"head")
        self.bodyhub.wait_walking_done()
        # 放置货物
        self.place()

        # ------------------放货部分测试------------------

        print('finish')
   


    

if __name__ == '__main__':
    # 注意：先检查更具体的情况（debug head），否则通用的 debug 分支会先被匹配到
    if len(sys.argv) >= 3 and (sys.argv[1] == 'debug') and (sys.argv[2] == 'head'):   # 校准头部摄像头位置
        TaskCarryConvert().set_target_pos('head')
    elif len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):         # 默认校准下巴摄像头位置
        TaskCarryConvert().set_target_pos('chin')
    else:
        TaskCarryConvert().start()

