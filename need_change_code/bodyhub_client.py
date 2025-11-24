#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from std_msgs.msg import *
from sensor_msgs.msg import *

from bodyhub.msg import *
from bodyhub.srv import *


class BodyhubClient(object):
    def __init__(self, id):
        self.ctl_id = id
        self.__curr_id = 0
        self.__is_walking = 0
        self.__timeout = 20  # s
        self.__step_len_max = [0.1, 0.05, 10.0]  # x, y ,theta

        self.__joint_ctl_pub = rospy.Publisher('MediumSize/BodyHub/jointControl', JointState, queue_size=1000)
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        self.__HeadJointPub = rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint, queue_size=100)

        rospy.Subscriber('/requestGaitCommand', Bool, self.__request_gait_cb)
        rospy.Subscriber("/MediumSize/BodyHub/WalkingStatus", Float64, self.__walk_state_cb, queue_size=2)

        self.__state_init = 20
        self.__state_preReady = 21
        self.__state_ready = 22
        self.__state_running = 23
        self.__state_pause = 24
        self.__state_stoping = 25
        self.__state_error = 26
        self.__state_directOperate = 27
        self.__state_walking = 28
        self.request_stop = False

    def __request_gait_cb(self, msg):
        pass

    def __walk_state_cb(self, msg):
        self.__is_walking = msg.data

    def __get_ctlid(self):
        try:
            rospy.wait_for_service('MediumSize/BodyHub/GetMasterID', self.__timeout)
        except:
            print ('error: wait_for_service GetMasterID!')
            return None
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
        response = client('get')
        return response.data

    def __get_status(self):
        try:
            rospy.wait_for_service('MediumSize/BodyHub/GetStatus', self.__timeout)
        except:
            print ('error: wait_for_service GetStatus!')
            return None
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)
        response = client('get')
        return response

    def __set_status(self, ctl_id, status):
        try:
            rospy.wait_for_service('MediumSize/BodyHub/StateJump',  self.__timeout)
        except:
            print ('error: wait_for_service StateJump!')
            return None
        client = rospy.ServiceProxy('MediumSize/BodyHub/StateJump', SrvState)
        response = client(ctl_id, status)
        # print("set_status: requested %s, got %s" % (status, response.stateRes))
        return response.stateRes

    def __check_id(self):
        self.__curr_id = self.__get_ctlid()
        if self.__curr_id == None:
            return -1
        if self.__curr_id == 0 or self.__curr_id == self.ctl_id:
            self.__curr_id = self.ctl_id
            return True
        else:
            rospy.logwarn('bodyhub busy, id is %d' % (self.__curr_id))
            return False

    def reset(self, root=False):
        # print("reset() called")
        status = self.__get_status()
        # print("in reset() current status:", status.data if status != None else "None")
        if status == None:
            return -1
        if status.data == 'preReady':
            return True
        if self.__check_id() == True or root == True:
            result = self.__set_status(self.__curr_id, 'stop')
            if result == None:
                return -2
            result = self.__set_status(self.__curr_id, 'reset')
            if result == None:
                return -3
            if result == self.__state_preReady:
                return True
        return False

    def ready(self):
        if self.__check_id() != True:
            return False
        # print("body hub cli: ", self.ctl_id)
        status = self.__get_status()
        # print("in ready() current status:", status.data if status != None else "None")
        if status == None:
            return -1
        if status.data == 'ready' or status.data == 'pause':
            return True  
        # print("status is not ready, try to set ready")          
        if self.reset() == True:
            result = self.__set_status(self.ctl_id, 'setStatus')
            # print("set ready result= ready = ",result)
            if result == None:
                return -2
            if result == self.__state_ready:
                return True
        return False

    def walk(self):
        # print("walk() called")
        if self.__check_id() != True:
            # print("__check_id failed")
            return False
        status = self.__get_status()
        # print("in walk() current status:", status.data if status != None else "None")
        if status == None:
            return -1
        if status.data == 'walking':
            # print("status is walking")
            return True
        if self.ready() == True:
            result = self.__set_status(self.ctl_id, 'walking')
            # print("set walking result=walking  = ",result)
            if result == None:
                return -2
            if result == self.__state_walking:
                return True
        return False
    



    # 获取指定ids的关节位置
    def get_joint_position(self, ids):
        try:
            rospy.wait_for_service('MediumSize/BodyHub/GetJointAngle', self.__timeout)
        except:
            print ('error: wait_for_service GetJointAngle!')
            return None
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetJointAngle', SrvServoAllRead)
        response = client(ids, 22)
        return response.getData
    
    def set_head_position(self, angles):
        """set head servos angle

        :param angles:[pan, tilt]
        :return:
        """
        if not rospy.core.is_shutdown_requested():
            self.__HeadJointPub.publish(positions=angles, mainControlID=self.ctl_id)
   
    #  设置指定关节ids的角度/位置。     
    def set_joint_position(self, ids, position):
        ids_str = [str(i) for i in ids]
        ids_str.append(str(self.ctl_id))
        self.__joint_ctl_pub.publish(name=ids_str, position=position)

    def wait_head_done(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service("MediumSize/BodyHub/GetHeadQueueSize")
                client = rospy.ServiceProxy("MediumSize/BodyHub/GetHeadQueueSize", SrvString)
                posesize = client("get_head_status").poseQueueSize
                if posesize < 2:
                    break
            except Exception as e:
                rospy.logerror("error ",e)
            rate.sleep()
                
    def wait_joint_done(self, completely=False):
        while not rospy.is_shutdown():
            if completely:
                if self.__get_status().data == 'pause':
                    break
            else:
                if self.__get_status().jointQueueSize <= 50:
                    break

    def walking(self, delta_x, delta_y, theta, timeout = None):
        try:
            rospy.wait_for_message('/requestGaitCommand', Bool, self.__timeout if timeout is None else timeout)
            # ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
            # current_angles = self.bodyhub.get_joint_position(ids)
            # print("Current Joint Angles: %s",current_angles)
        except:
            rospy.logwarn("wait_for_service requestGaitCommand timeout!")
            result = 1
        else:
            result = 0
        self.__is_walking = 1
        self.__gait_cmd_pub.publish(data=[delta_x, delta_y, theta])
        return result

    def wait_walking_done(self):
        while not rospy.is_shutdown():
            if self.__is_walking == 0:
                result = self.__set_status(self.ctl_id, 'pause')
                break
        

    def walking_the_distance(self, delta_x, delta_y, theta):
        direction = [0, 0, 0]
        direction[0] = 1 if delta_x >= 0 else -1
        direction[1] = 1 if delta_y >= 0 else -1
        direction[2] = 1 if theta >= 0 else -1

        xNumberOfStep = abs(delta_x)//self.__step_len_max[0]
        yNumberOfStep = abs(delta_y)//self.__step_len_max[1]
        aNumberOfStep = abs(theta)//self.__step_len_max[2]
        delta_x = delta_x - ((self.__step_len_max[0] * xNumberOfStep)*direction[0])
        delta_y = delta_y - ((self.__step_len_max[1] * yNumberOfStep)*direction[1])
        theta = theta - ((self.__step_len_max[2] * aNumberOfStep)*direction[2])
        numberOfStep = max(xNumberOfStep, yNumberOfStep, aNumberOfStep)

        for i in range(0, int(numberOfStep+1)):
            if self.request_stop:
                self.reset_walking_flag()
                break
            if xNumberOfStep >= 1:
                x = self.__step_len_max[0]*direction[0]
                xNumberOfStep = xNumberOfStep - 1
            elif xNumberOfStep == 0:
                x = delta_x
                xNumberOfStep = xNumberOfStep - 1
            else:
                x = 0.0

            if yNumberOfStep >= 1:
                y = self.__step_len_max[1]*direction[1]
                yNumberOfStep = yNumberOfStep - 1
            elif yNumberOfStep == 0:
                y = delta_y
                yNumberOfStep = yNumberOfStep - 1
            else:
                y = 0.0

            if aNumberOfStep >= 1:
                a = self.__step_len_max[2]*direction[2]
                aNumberOfStep = aNumberOfStep - 1
            elif aNumberOfStep == 0:
                a = theta
                aNumberOfStep = aNumberOfStep - 1
            else:
                a = 0.0
            self.walking(x, y, a)

    def walking_n_steps(self, cmd, n):
        for i in range(0, n):
            if self.walking(cmd[0], cmd[1], cmd[2]):
                return 1
        return 0

    def stop_walking(self):
        if self.__is_walking != 0:
            self.request_stop = True

    def reset_walking_flag(self):
        self.request_stop = False
