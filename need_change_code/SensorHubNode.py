#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from sys import path
import os
import time
import yaml
from std_msgs.msg import *

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Range
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import Illuminance
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity
from geometry_msgs.msg import PoseStamped
import math
from tf.transformations import quaternion_from_euler

from bodyhub.msg import SensorRawData

SensorNameIDFilePath = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config/sensorNameID.yaml"))
SensorIdNameDict = {}
SensorList = []
gravity_acc = 9.8
ImuPub = rospy.Publisher('MediumSize/SensorHub/Imu', Imu, queue_size=1)
MagneticFieldPub = rospy.Publisher('MediumSize/SensorHub/MagneticField', MagneticField, queue_size=1)
BatteryStatePub = rospy.Publisher('MediumSize/SensorHub/BatteryState', BatteryState, queue_size=1)
RangePub = rospy.Publisher('MediumSize/SensorHub/Range', Range, queue_size=1)
ChannelFloat32Pub = rospy.Publisher('MediumSize/SensorHub/sensor_CF1', ChannelFloat32, queue_size=1)
IlluminancePub = rospy.Publisher('MediumSize/SensorHub/Illuminance', Illuminance, queue_size=1)
TemperaturePub = rospy.Publisher('MediumSize/SensorHub/Temperature', Temperature, queue_size=1)
HumidityPub = rospy.Publisher('MediumSize/SensorHub/Humidity', RelativeHumidity, queue_size=1)
# TTT
# Publish IMU-derived orientation as a PoseStamped (frame_id left to user; set to 'imu_link' by default)
ImuPosePub = rospy.Publisher('MediumSize/SensorHub/ImuPose', PoseStamped, queue_size=1)
class JY901_Imudata:
    def __init__(self):
        self.update_time=time.time()
        self.data = [0, 0, 0,
                     0, 0, 0,
                     0, 0, 0]
        self.same_data_count = 0
        self.available=False
        
    def push_back(self,data):
        self.update_time=time.time()
        data = list(data)
        self.same_data_count = 0 if self.data != data else self.same_data_count + 1
        self.available = False if self.same_data_count > 50 else True
        data[3],data[4],data[5] = data[3]*gravity_acc,data[4]*gravity_acc,data[5]*gravity_acc
        self.data = data

    def is_update(self): 
        return False if time.time() - self.update_time > 0.1 else True
    
    def is_available(self):
        return self.is_update() and self.available
    
class Sensor:
    def __init__(self, name, id, addr, length, data):
        self.name = name
        self.id = id
        self.dataAddr = addr
        self.dataLength = length
        self.rawData = data

    def getData(self):
        return self.rawData


def UintToInt(num, bit):
    bit -= 1
    if (num & (1 << bit)):
        temp = ~(num - 1)
        return -(temp & ((1 << bit) - 1))
    return num

def jy901callback(msg):
    # print(msg.data)
    jy901_data.push_back(msg.data)
    if jy901_data.is_available():# 如果jy901数据可用,则直接转发到imu topic
        imuData = Imu()
        imuData.angular_velocity.x = jy901_data.data[0]
        imuData.angular_velocity.y = jy901_data.data[1]
        imuData.angular_velocity.z = jy901_data.data[2]
        imuData.linear_acceleration.x = jy901_data.data[3]
        imuData.linear_acceleration.y = jy901_data.data[4]
        imuData.linear_acceleration.z = jy901_data.data[5]
        # TTT
        # If the JY901 provides angle data (typically roll, pitch, yaw in degrees) at indices 6,7,8,
        # convert to radians and form a quaternion. This fills the Imu.orientation field and also
        # publishes a PoseStamped for convenience.
        try:
            if len(jy901_data.data) >= 9:
                # Assume data[6]=roll, data[7]=pitch, data[8]=yaw in degrees
                roll = math.radians(jy901_data.data[6])
                pitch = math.radians(jy901_data.data[7])
                yaw = math.radians(jy901_data.data[8])
                q = quaternion_from_euler(roll, pitch, yaw)  # returns [x,y,z,w]
                imuData.orientation.x = q[0]
                imuData.orientation.y = q[1]
                imuData.orientation.z = q[2]
                imuData.orientation.w = q[3]

                # Optionally publish a PoseStamped with this orientation
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'imu_link'
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.position.z = 0.0
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                ImuPosePub.publish(pose)
        except Exception as e:
            rospy.logwarn('Failed to compute IMU orientation from JY901 data: %s' % str(e))
            # TTT
        ImuPub.publish(imuData)

    
# 板载imu
def ImuDataProcess(rawData):
    if not jy901_data.is_available():
        # print("using imudata")
        imuData = Imu()
        grayCoefficient = 1000.0 / 32768
        accCoefficient = 8.0 * gravity_acc / 32768

        imuData.angular_velocity.x = UintToInt((rawData[1] << 8) + rawData[0], 16)
        imuData.angular_velocity.y = UintToInt((rawData[3] << 8) + rawData[2], 16)
        imuData.angular_velocity.z = UintToInt((rawData[5] << 8) + rawData[4], 16)

        imuData.angular_velocity.x *= grayCoefficient
        imuData.angular_velocity.y *= grayCoefficient
        imuData.angular_velocity.z *= grayCoefficient

        imuData.linear_acceleration.x = UintToInt((rawData[7] << 8) + rawData[6], 16)
        imuData.linear_acceleration.y = UintToInt((rawData[9] << 8) + rawData[8], 16)
        imuData.linear_acceleration.z = UintToInt((rawData[11] << 8) + rawData[10], 16)

        imuData.linear_acceleration.x *= accCoefficient
        imuData.linear_acceleration.y *= accCoefficient
        imuData.linear_acceleration.z *= accCoefficient

        imuData.angular_velocity.x, imuData.angular_velocity.y, imuData.angular_velocity.z = \
            imuData.angular_velocity.y, imuData.angular_velocity.x, -imuData.angular_velocity.z
        imuData.linear_acceleration.x, imuData.linear_acceleration.y, imuData.linear_acceleration.z = \
            imuData.linear_acceleration.y, imuData.linear_acceleration.x, -imuData.linear_acceleration.z
        ImuPub.publish(imuData)

    else:# 阻断旧的内置imu数据
        pass
        # print("using jy901data")



# 板载磁场
def MagneticDataProcess(rawData):

    magCoefficient = 50.0 / 32768
    magData = MagneticField()

    magData.magnetic_field.x = UintToInt((rawData[1] << 8) + rawData[0], 16)
    magData.magnetic_field.y = UintToInt((rawData[3] << 8) + rawData[2], 16)
    magData.magnetic_field.z = UintToInt((rawData[5] << 8) + rawData[4], 16)

    magData.magnetic_field.x *= magCoefficient
    magData.magnetic_field.y *= magCoefficient
    magData.magnetic_field.z *= magCoefficient

    MagneticFieldPub.publish(magData)


# 板载adc
def AdcDataProcess(rawData):
    battData = BatteryState()
    battData.voltage = rawData[0] / 10.0
    battData.present = True
    BatteryStatePub.publish(battData)


# 板载按键
def KeyDataProcess(rawData):
    status = 0
    for i in range(5):
        if rawData[i]:
            status |= (1 << i)
    if status > 0:
        ChannelFloat32Pub.publish(name="KeyStatus", values=[status])


# 板载led
def LedDataProcess(rawData):
    ChannelFloat32Pub.publish(name="LedStatus", values=[(rawData[0] << 8) + rawData[1]])


# 板载测距
def RangeDataProcess(rawData):
    RangePub.publish(range=(rawData[1] << 8) + rawData[0])


# 板载传感器
def BaseBoardDataProcess(rawData):
    ledStatus = []
    keyStatus = []
    imuData = []
    MagneticData = []
    adcData = []
    rangeData = []
    startAddr = 24

    offset, lenght = 26, 2
    for i in range(offset, offset + lenght):
        ledStatus.append(rawData[i - startAddr])
    LedDataProcess(ledStatus)

    offset, lenght = 30, 5
    for i in range(offset, offset + lenght):
        keyStatus.append(rawData[i - startAddr])
    KeyDataProcess(keyStatus)

    offset, lenght = 38, 12
    for i in range(offset, offset + lenght):
        imuData.append(rawData[i - startAddr])
    ImuDataProcess(imuData)

    offset, lenght = 50, 1
    for i in range(offset, offset + lenght):
        adcData.append(rawData[i - startAddr])
    AdcDataProcess(adcData)

    offset, lenght = 51, 6
    for i in range(offset, offset + lenght):
        MagneticData.append(rawData[i - startAddr])
    MagneticDataProcess(MagneticData)

    offset, lenght = 59, 2
    for i in range(offset, offset + lenght):
        rangeData.append(rawData[i - startAddr])
    RangeDataProcess(rangeData)


# 光照度传感器
def IlluminanceDataProcess(rawData):
    IlluminancePub.publish(illuminance=rawData[0], variance=0)


# 温湿度传感器
def HumitureDataProcess(rawData):
    temperature = rawData[0] + rawData[1]
    humidity = rawData[2] + rawData[3]
    TemperaturePub.publish(temperature=temperature, variance=0)
    HumidityPub.publish(relative_humidity=humidity, variance=0)


# 人体红外传感器
def InfraredDataProcess(rawData):
    ChannelFloat32Pub.publish(name="InfraredStatus", values=[rawData[0]])


# 颜色传感器
def ColorDataProcess(rawData):
    ChannelFloat32Pub.publish(name="ColorValue", values=[rawData[0]])


# 火焰传感器
def FireDataProcess(rawData):
    ChannelFloat32Pub.publish(name="FireStatus", values=[rawData[0]])


# 气体传感器
def GasDataProcess(rawData):
    ChannelFloat32Pub.publish(name="GasStatus", values=[rawData[0]])


# 触摸传感器
def TouchDataProcess(rawData):
    ChannelFloat32Pub.publish(name="TouchStatus", values=[rawData[0]])


def SensorRawDataProcess():
    while (len(SensorList) > 0):
        sensor = SensorList.pop(0)
        if SensorIdNameDict[sensor.id] == 'baseBoard':
            BaseBoardDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'illuminance':
            IlluminanceDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'humiture':
            HumitureDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'infrared':
            InfraredDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'color':
            ColorDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'fire':
            FireDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'gas':
            GasDataProcess(sensor.getData())
        elif SensorIdNameDict[sensor.id] == 'touch':
            TouchDataProcess(sensor.getData())
        else:
            pass


# ----------------------------------------------------------------------------------------


def LoadYamlFile(filePath):
    yamlFile = open(filePath)
    sensorNameIdDoc = yaml.load(yamlFile)

    for name in sensorNameIdDoc["sensorNameID"]:
        SensorIdNameDict[sensorNameIdDoc["sensorNameID"][name]] = name
    rospy.loginfo(SensorIdNameDict)


def SensorRawDataCallback(rawData):
    # rospy.loginfo(rawData.sensorData)
    sensorCount = rawData.sensorCount
    # dataOverallLength = rawData.dataLength

    dataIndex = 0
    sensorIdList = list(rawData.sensorReadID)
    for i in range(sensorCount):
        if sys.version<'3':
            
            sensorIdList[i] = ord(sensorIdList[i])
        if sensorIdList[i] in SensorIdNameDict:
            SensorList.append(
                Sensor(SensorIdNameDict[sensorIdList[i]], sensorIdList[i], rawData.sensorStartAddress[i], rawData.sensorReadLength[i],
                       rawData.sensorData[dataIndex:(dataIndex + rawData.sensorReadLength[i])]))
        else:
            rospy.loginfo('Undefined id')
        dataIndex += rawData.sensorReadLength[i]

    SensorRawDataProcess()


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node('SensorHubNode', anonymous=True, log_level=rospy.INFO)  # DEBUG INFO ERROR WARN
        print('Starting SensorHubNode node')
        jy901_data=JY901_Imudata()
        LoadYamlFile(SensorNameIDFilePath)

        rospy.Subscriber("MediumSize/BodyHub/SensorRaw", SensorRawData, SensorRawDataCallback)
        jy901sub=rospy.Subscriber("/jy901Module_node/jy901Data",Float64MultiArray,jy901callback,queue_size=1)

        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down SensorHub node.")
