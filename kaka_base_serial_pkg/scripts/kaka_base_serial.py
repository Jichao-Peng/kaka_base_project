#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import serial
import numpy as np
import tf
import tf_conversions
import nav_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class KakaBaseSerial:
    def __init__(self):
        # ros config
        self.__tf_broadcaster = tf.TransformBroadcaster()
        self.__odometry_pub = rospy.Publisher('odom',nav_msgs.msg.Odometry,queue_size=50)
        self.__robot_control_pub = rospy.Publisher('robot_control',std_msgs.msg.String,queue_size=50)
        self.__cmd_vel_sub = rospy.Subscriber("cmd_vel",geometry_msgs.msg.Twist,self.CmdVelCallBack)
        self.__face_detection_result_sub = rospy.Subscriber("face_detection_result",std_msgs.msg.Int32,self.FaceDetectionResultCallBack)
        self.__base_move_result_sub = rospy.Subscriber("base_move_result",std_msgs.msg.Int32,self.BaseMoveResultCallBack)

        # serial config
        self.__serial = serial.Serial('/dev/ttyUSB0',115200,timeout=1)

        # parameter config
        self.__t = (27/(0.05*2*math.pi))
        self.__h = (2*math.pi*0.05/2700)
        self.__k = 0.4775
        self.__f = np.array([[0.25,0.25,0.25,0.25],
                             [-0.25,0.25,-0.25,0.25],
                             [-0.25/self.__k,-0.25/self.__k,0.25/self.__k,0.25/self.__k]])
        self.__send_buff = []
        self.__recv_buff = []
        self.__data_buff = []
        self.__analysis_state = "READY" #解析状态机:HEAD TYPE LENGTH DATA TAIL READY
        self.__control_cmd_type = "UNKNOW" #控制状态机:
        self.__control_cmd_data_lenth = 0

        self.__current_encode = np.array([0, 0, 0, 0],dtype=np.int32)
        self.__previous_encode = np.array([0, 0, 0, 0],dtype=np.int32)
        self.__encode_diff = np.array([0, 0, 0, 0],dtype=np.int32)
        self.__base_distance = np.array([0, 0, 0],dtype=np.double)
        self.__base_distance_diff = np.array([0, 0, 0], dtype=np.double)
        self.__base_velocity = np.array([0, 0, 0], dtype=np.double)
        self.__current_time = rospy.Time.now().to_sec()
        self.__previous_time = rospy.Time.now().to_sec()
        self.__heartbeat_count = 5

    #运行整个系统
    def Run(self):
        #open serial
        self.__serial.isOpen()
        #ros start running
        rospy.loginfo("KAKA_BASE_SERIAL running!")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            serial_data_unread = self.__serial.inWaiting()
            if serial_data_unread:
                serial_data = self.__serial.read(serial_data_unread)
                recv_data = [hex(ord(i)) for i in serial_data]
                self.PutDataIntoRecvBuff(recv_data)
            self.Decode()
            #self.SendHeartBeat()
            rate.sleep()

    #人脸识别结果
    def FaceDetectionResultCallBack(self,msg):
        if msg.data == 0:
            self.__serial.write("\x55\x05\x01\x00\xAA")
        elif msg.data == 1:
            self.__serial.write("\x55\x05\x01\x01\xAA")

    #底盘运动结果
    def BaseMoveResultCallBack(self,msg):
        if msg.data == 0:
            self.__serial.write("\x55\x03\x01\x00\xAA")#运动为到达目标位置，这个时候状态灯2闪烁
        elif msg.data == 1:
            self.__serial.write("\x55\x03\x01\x01\xAA")#运动到达目标位置，这个时候状态灯到2

    #发送速度
    def CmdVelCallBack(self,msg):
        px = math.fabs(self.__t*msg.linear.x)
        py = math.fabs(self.__t*msg.linear.y)
        pz = math.fabs(self.__t*msg.angular.z)
        #限幅
        if px > 0.6:
            px = 0.6
        if py > 0.6:
            py = 0.6
        if pz > 0.6:
            pz = 0.6
            
        t = 0
        if msg.linear.y<0:
            t = t|(1<<2)
        if msg.linear.x>0:
            t = t|(1<<1)
        if msg.angular.z<0:
            t = t|(1<<0)
        send_cmd = []
        send_cmd.append(0x55)
        send_cmd.append(0x01)
        send_cmd.append(0x08)
        send_cmd.append(0x01)
        send_cmd.append((int(py) >> 8) & 0xff) #底盘定义的坐标新和ros里面规定的坐标系不一样，底盘的坐标系应该是x朝前
        send_cmd.append((int(py)) & 0xff)
        send_cmd.append((int(px) >> 8) & 0xff)
        send_cmd.append((int(px)) & 0xff)
        send_cmd.append((int(pz) >> 8) & 0xff)
        send_cmd.append((int(pz)) & 0xff)
        send_cmd.append(int(t & 0xff))
        send_cmd.append(0xAA)
        self.__serial.write(bytearray(send_cmd))

    #发送心跳包
    def SendHeartBeat(self):
        if self.__heartbeat_count < 20:
            self.__heartbeat_count = 0
            self.__serial.write("\x55\x00\x00\xAA")
        else:
            self.__heartbeat_count += 1

    #解析数据
    def Decode(self):
        while self.IsDataInRecvBuff():
            data = self.GetDataInRecvBuff()

            # rospy.loginfo(data)
            #head
            if self.__analysis_state == "READY" or self.__analysis_state == "HEAD":
                #rospy.loginfo("KAKA_BASE_SERIAL head")
                if data == "0x55":
                    self.__analysis_state = "TYPE"
            #type
            elif self.__analysis_state == "TYPE":
                #rospy.loginfo("KAKA_BASE_SERIAL type")
                if data == "0x1":
                    self.__analysis_state = "LENTH"
                    self.__control_cmd_type = "ENCODER_CMD"
                elif data == "0x2":
                    self.__analysis_state = "LENTH"
                    self.__control_cmd_type = "START_MOVE_CMD"
                elif data == "0x3":
                    self.__analysis_state = "LENTH"
                    self.__control_cmd_type = "STOP_MOVE_CMD"
                elif data == "0x4":
                    self.__analysis_state = "LENTH"
                    self.__control_cmd_type = "START_FACE_DETECTION_CMD"
                elif data == "0x5":
                    self.__analysis_state = "LENTH"
                    self.__control_cmd_type = "STOP_FACE_DETECTION_CMD"
                elif data == "0x6":
                    self.__analysis_state = "LENTH"
                    self.__control_cmd_type = "START_FOLLOW_CMD"
                elif data == "0x7":
                    self.__analysis_state = "LENTH"
                    self.__control_cmd_type = "STOP_FOLLOW_CMD"
                else:
                    self.__analysis_state = "READY"
                    self.__control_cmd_type = "UNKNOW"
            #lenth
            elif self.__analysis_state == "LENTH":
                #rospy.loginfo("KAKA_BASE_SERIAL lenth")
                if data == "0x0":
                    self.__analysis_state = "TAIL"
                else:
                    self.__analysis_state = "DATA"
                    self.__control_cmd_data_lenth = int(data,16)
            #data
            elif self.__analysis_state == "DATA":
                #rospy.loginfo("KAKA_BASE_SERIAL data")
                if self.__control_cmd_type == "ENCODER_CMD":
                    if self.__control_cmd_data_lenth != 1:
                        self.__control_cmd_data_lenth -= 1
                        self.__data_buff.append(data)#if data lenth is not zero, put the data into a data buff
                    else:
                        self.__data_buff.append(data)
                        if len(self.__data_buff) == 16:
                            self.__current_time = rospy.Time.now().to_sec()#TODO: the time is wrong
                            dt = (self.__current_time-self.__previous_time)
                            #compute encode
                            for i in range(0,4):
                                self.__current_encode[i] = ((int(self.__data_buff[i*4],16))<<24 & 0xffffffff)|\
                                                           ((int(self.__data_buff[i*4+1],16))<<16 & 0xffffffff)|\
                                                           ((int(self.__data_buff[i*4+2],16))<<8 & 0xffffffff)|\
                                                           ((int(self.__data_buff[i*4+3],16)) & 0xffffffff)
                            self.__encode_diff = self.__current_encode - self.__previous_encode
                            #滤掉一些奇怪的值
                            for i in range(0,4):
                                if math.fabs(self.__encode_diff[i]) > 500:
                                    self.__encode_diff[i] = 0
                            #compute diffrential distance and velocity
                            self.__base_distance_diff = self.__f.dot(self.__encode_diff*self.__h)
                            self.__base_distance_diff[0] = -self.__base_distance_diff[0]

                            self.__base_velocity = self.__base_distance_diff/dt
                            #from the local coordinate system to the global coordinate system
                            delta_x = self.__base_distance_diff[0] * math.cos(self.__base_distance[2])\
                                      - self.__base_distance_diff[1] * math.sin(self.__base_distance[2])
                            delta_y = self.__base_distance_diff[0] * math.sin(self.__base_distance[2])\
                                      + self.__base_distance_diff[1] * math.cos(self.__base_distance[2])
                            delta_theta = self.__base_distance_diff[2]
                            #integral
                            self.__base_distance[0] += delta_x
                            self.__base_distance[1] += delta_y
                            self.__base_distance[2] += delta_theta

                            rospy.loginfo(self.__base_distance)
                            #backset
                            self.__previous_time = self.__current_time
                            self.__previous_encode = self.__current_encode.copy()
                            #clear data bufe
                            self.__data_buff[:] = []

                            self.__analysis_state = "TAIL"
                        else:
                            self.__data_buff[:] = []
                            self.__analysis_state = "READY"
                else:
                    self.__analysis_state = "TAIL"
            #TAIL
            elif self.__analysis_state == "TAIL":
                #rospy.loginfo("KAKA_BASE_SERIAL tail")
                if data == "0xaa":
                    # rospy.loginfo("receive data")
                    # rospy.loginfo(" ")
                    if self.__control_cmd_type == "ENCODER_CMD":
                        # pulish tf
                        self.__tf_broadcaster.sendTransform((self.__base_distance[0],self.__base_distance[1],0),
                                                            tf_conversions.transformations.quaternion_from_euler(0, 0,self.__base_distance[2]),
                                                            rospy.Time.now(),
                                                            "base_link",
                                                            "odom")
                        odometry = nav_msgs.msg.Odometry()
                        odometry.header.stamp = rospy.Time.now()
                        odometry.header.frame_id = "odom"
                        odometry.pose.pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(self.__base_distance[0],self.__base_distance[1],0),
                                                                    geometry_msgs.msg.Quaternion(*(tf_conversions.transformations.quaternion_from_euler(0, 0, self.__base_distance[2]))))
                        odometry.child_frame_id = "base_link"
                        odometry.twist.twist = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(self.__base_velocity[0],self.__base_velocity[1],0),
                                                                       geometry_msgs.msg.Vector3(0,0,self.__base_velocity[2]))
                        self.__odometry_pub.publish(odometry)#pulish odometry
                        self.__analysis_state = "READY"
                    elif self.__control_cmd_type == "START_MOVE_CMD":
                        self.__robot_control_pub.publish("start_move")
                        self.__analysis_state = "READY"
                        self.__serial.write("\x55\x02\x00\xAA")
                    elif self.__control_cmd_type == "STOP_MOVE_CMD":
                        self.__robot_control_pub.publish("stop_move")
                        self.__analysis_state = "READY"
                        self.__serial.write("\x55\x03\x00\xAA")
                    elif self.__control_cmd_type == "START_FACE_DETECTION_CMD":
                        self.__robot_control_pub.publish("start_face_detection")
                        self.__analysis_state = "READY"
                        self.__serial.write("\x55\x04\x00\xAA")
                    elif self.__control_cmd_type == "STOP_FACE_DETECTION_CMD":
                        self.__robot_control_pub.publish("stop_face_detection")
                        self.__analysis_state = "READY"
                    elif self.__control_cmd_type == "START_FOLLOW_CMD":
                        self.__robot_control_pub.publish("start_follow_move")
                        self.__analysis_state = "READY"
                        self.__serial.write("\x55\x06\x00\xAA")
                    elif self.__control_cmd_type == "STOP_FOLLOW_CMD":
                        self.__robot_control_pub.publish("stop_follow_move")
                        self.__analysis_state = "READY"
                        self.__serial.write("\x55\x07\x00\xAA")
                else:
                    self.__analysis_state = "READY"





    ##-----function for send buff and receive buff-----
    #put data into recevie buff
    def PutDataIntoRecvBuff(self,list):
        self.__recv_buff.extend(list)

    #judge if there is data in receive buff
    def IsDataInRecvBuff(self):
        if self.__recv_buff:
            return True
        else:
            return False

    #get the first data in the receive buff
    def GetDataInRecvBuff(self):
        data = self.__recv_buff[0]
        self.__recv_buff.pop(0)
        return data

    #put data into recevie buff
    def PutDataIntoSendBuff(self,list):
        self.__send_buff.extend(list)

    #judge if there is data in receive buff
    def IsDataInSendBuff(self):
        if self.__send_buff:
            return True
        else:
            return False

    #get the first data in the receive buff
    def GetDataInSendBuff(self):
        data = self.__send_buff[0]
        self.__send_buff.pop(0)
        return data



if __name__ == '__main__':
    try:
        rospy.init_node('kaka_base_serial', anonymous=True)
        kaka_base_serial = KakaBaseSerial()
        kaka_base_serial.Run()
    except serial.SerialException:
        rospy.loginfo("KAKA_BASE_SEIRAL open serial failure!")




