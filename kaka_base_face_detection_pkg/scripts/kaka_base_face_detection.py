#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import cv2
import rospy
import cv_bridge
import numpy as np
import std_msgs.msg
import sensor_msgs.msg
from PIL import Image#PIL库,python上集成的图像图例库
import face_recognition


#主成分分析法识别人脸
class PCAFaceRecognize():
    def __init__(self,master_face_list):
        self.__face_matrix = self.CreatDataBase(master_face_list)
        self.__face_core = self.EigenFaceCore(self.__face_matrix)

    def CreatDataBase(self,image_list):
        image_matrix = []
        for image in image_list:
            image = Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
            image = image.resize((40,40))
            gray_image = image.convert('L')
            image_array = list(gray_image.getdata()) #转化为一维数组
            image_matrix.append(image_array)

        image_matrix = np.array(image_matrix) #转化为二维矩阵,图像像素按照行排列，每列是一幅图
        return image_matrix

    def EigenFaceCore(self,image_matrix):
        train_number,size = np.shape(image_matrix) #返回图像个数，和图像大小
        mean_array = image_matrix.mean(0) #按列计算平均值
        diff_matrix = image_matrix - mean_array #计算差值
        diff_matrix = np.mat(diff_matrix)
        eigen_values,eigen_vectors = np.linalg.eig(diff_matrix*diff_matrix.T) #求得特征向量和特征值

        #此时得到的特征值和特征向量并无顺序
        eigen_vectors = list(eigen_vectors.T)

        eigen_vectors = np.array(eigen_vectors) #由于无法直接创建一维矩阵，所以需要一个数组过度
        eigen_vectors = np.mat(eigen_vectors).T

        #计算特征脸
        eigen_faces = diff_matrix.T*eigen_vectors
        return eigen_faces

    def Recognize(self,unknow_image):
        mean_array = self.__face_matrix.mean(0)
        diff_matrix = self.__face_matrix - mean_array
        size,train_number = np.shape(self.__face_core) #确定过滤后的图片数目
        projected_image = self.__face_core.T * diff_matrix.T #将每个样本投影到特征空间

        #预处理测试图片，投影到特征空间上
        unknow_image = Image.fromarray(cv2.cvtColor(unknow_image, cv2.COLOR_BGR2RGB))
        unknow_image = unknow_image.resize((40,40))
        gray_unknow_image = unknow_image.convert('L')
        unknow_image_array = list(gray_unknow_image.getdata())
        unknow_image_array = np.array(unknow_image_array)
        diff_unknow_matrix = unknow_image_array - mean_array

        #转化成矩阵方便接下来的乘法操作
        diff_unknow_matrix = np.array(diff_unknow_matrix)
        diff_unknow_matrix = np.mat(diff_unknow_matrix)

        projected_unknow_image = self.__face_core.T * diff_unknow_matrix.T #将待测图片投影到特征空间

        #按照欧式距离计算最匹配的人脸
        distance = []
        for i in range(0,train_number):
            temp = np.linalg.norm(projected_unknow_image - projected_image[:,i])
            rospy.loginfo(temp)
            distance.append(temp)

        min_distance = min(distance)
        index = distance.index(min_distance)
        return index+1


class KakaBaseFaceDetection:
    def __init__(self):
        #CNN初始化模板
        # 读取模板数量
        txtPath = "/home/leo/Desktop/Kaka_Base_Project/src/kaka_base_face_detection_pkg/templates/template_nums.txt"  # 存放模板数量
        txtFile = open(txtPath, 'rb')
        templateNums = int(txtFile.read().decode('gbk'))  # 样本模板数量
        txtFile.close()

        # 读取模板文件
        templatesPath = "/home/leo/Desktop/Kaka_Base_Project/src/kaka_base_face_detection_pkg/templates/templates"
        self.__template_encodings=[]
        for i in range(6):
            f=np.load(templatesPath+str(i)+'.npy')
            self.__template_encodings.append(f)
        rospy.loginfo(self.__template_encodings)

        # 读取标签文件
        labelsPath = "/home/leo/Desktop/Kaka_Base_Project/src/kaka_base_face_detection_pkg/templates/labels"
        self.__labels = []
        for i in range(6):
            f=np.load(labelsPath+str(i)+'.npy')
            self.__labels.append(f)
        rospy.loginfo(self.__labels)

        # 记录判断的帧数，当一定帧数中为识别出人脸时取消检测
        self.__frame_cnt = 0
        self.__frame_detect_cnt = 0

        '''
        #先读入参数，使用级联表初始化haar特征检测器
        cascade_1_path = "/home/leo/Desktop/Kaka_Base_Project/src/kaka_base_face_detection_pkg/config/haarcascade_frontalface_alt.xml"
        cascade_2_path = "/home/leo/Desktop/Kaka_Base_Project/src/kaka_base_face_detection_pkg/config/haarcascade_profileface.xml"
        if not os.path.isfile(cascade_1_path):
            raise RuntimeError("%s: not found" % cascade_1_path)
        else:
            self.__cascade_1 = cv2.CascadeClassifier(cascade_1_path)
        if not os.path.isfile(cascade_2_path):
            raise RuntimeError("%s: not found" % cascade_2_path)
        else:
            self.__cascade_2 = cv2.CascadeClassifier(cascade_2_path)
        '''
        '''
        #生成数据库
        master_number = 3
        master_image_path = "/home/leo/Desktop/Kaka_Base_Project/src/kaka_base_face_detection_pkg/data"
        master_face_list = []
        for i in range(1, master_number + 1):
            master_image = cv2.imread(master_image_path + "/" + str(i) + '.jpg')
            master_face_list.append(master_image)
            rospy.loginfo("KAKA_BASE_FACE_DETECTION master_id:"+str(i))

        self.__face_recognize = PCAFaceRecognize(master_face_list)
        rospy.loginfo("KAKA_BASE_FACE_DETECTION master'database create success!")
        '''

        # 创建ros相关
        self.__cv_bridge = cv_bridge.CvBridge()
        self.__face_detection_image_pub = rospy.Publisher("face_detection_image", sensor_msgs.msg.Image,
                                                          queue_size=9)
        self.__face_detection_image_sub = rospy.Subscriber("/camera/rgb/image_raw", sensor_msgs.msg.Image,
                                                           self.ImageCallBack,
                                                           queue_size=9)
        self.__robot_control_sub = rospy.Subscriber("robot_control",std_msgs.msg.String,self.RobotControlCallBack,queue_size=50)
        self.__face_detection_result_pub = rospy.Publisher("face_detection_result",std_msgs.msg.Int32,queue_size=50)
        self.__face_detection_flag = False

    def Run(self):
        rospy.loginfo("KAKA_BASE_FACE_DETECTION running!")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

    def RobotControlCallBack(self,msg):
        if msg.data == "start_face_detection":
            rospy.loginfo("KAKA_BASE_FACE_DETECTION start detecting!")
            self.__face_detection_flag = True
        elif msg.data == "stop_face_detection":
            rospy.loginfo("KAKA_BASE_FACE_DETECTION stop detecting!")
            self.__face_detection_flag = False

    def ImageCallBack(self, image):
        if self.__face_detection_flag == True:
            self.__frame_cnt += self.__frame_cnt
            rospy.loginfo("KAKA_BASE_FACE_DETECTION detecting!")
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                frame = self.__cv_bridge.imgmsg_to_cv2(image, "bgr8")
            except cv_bridge.CvBridgeError as e:
                print e
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

            face_locations = face_recognition.face_locations(small_frame)
            face_encodings = face_recognition.face_encodings(small_frame, face_locations)
            rospy.loginfo("KAKA_BASE_FACE_DETECTION encoding!")
            face_names = []
            for face_encoding in face_encodings:
                results = face_recognition.compare_faces(self.__template_encodings, face_encoding, tolerance=0.5)

                # TODO 这里忽略了一个问题，那就是识别到多个都是True的情况
                if True in results:
                    for i in range(0, len(results)):
                        if results[i] == True:
                            face_name = self.__labels[i]

                            #检测到用户人脸，发布结果
                            self.__face_detection_result_pub.publish(1)
                            self.__frame_cnt = 0
                else:
                    face_name = 'unknown'
                face_names.append(face_name)
                rospy.loginfo(face_names)

            for (top, right, bottom, left), name in zip(face_locations, face_names):
                # 之前缩小了4倍，现在放大4倍
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4

                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), 2)
                cv2.putText(frame, str(name), (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 255, 255), 2)

            rospy.loginfo("KAKA_BASE_FACE_DETECTION pulishing!")
            self.__face_detection_image_pub.publish(self.__cv_bridge.cv2_to_imgmsg(frame, "bgr8"))

            # 未检测到用户人脸，发布结果
            if self.__frame_cnt > 20:
                self.__face_detection_result_pub.publish(0)
                self.__frame_cnt = 0

    '''
    #TODO：目前采用的算法是用opencv的haar算法检测人脸，然后用主成分分析法识别人脸
    def ImageCallBack(self,image):
        if self.__face_detection_flag == True:
            rospy.loginfo("KAKA_BASE_FACE_DETECTION detecting!")
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                cv_image = self.__cv_bridge.imgmsg_to_cv2(image, "bgr8")
            except cv_bridge.CvBridgeError as e:
                print e
            frame = np.array(cv_image, dtype=np.uint8)
            # 创建灰度图像
            grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 创建平衡直方图，减少光线影响
            grey_image = cv2.equalizeHist(grey_image)
            # 尝试检测人脸
            faces_result = self.DetecFace(grey_image)

            # 在opencv的窗口中框出所有人脸区域
            if len(faces_result) > 0:
                for face in faces_result:
                    x, y, w, h = face
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (50, 255, 50), 2)
                    master_id = self.__face_recognize.Recognize(cv_image[x:x+w,y:y+h])#TODO：这个人脸识别函数不太好用
                    self.__face_detection_result_pub.publish(1)
            # 将识别后的图像转换成ROS消息并发布
            self.__face_detection_image_pub.publish(self.__cv_bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def DetecFace(self, input_image):
        # 首先匹配正面人脸的模型
        faces = []
        if self.__cascade_1:
            faces = self.__cascade_1.detectMultiScale(input_image,1.2,2,cv2.CASCADE_SCALE_IMAGE,(40, 60))
        # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
        if len(faces) == 0 and self.__cascade_2:
            faces = self.__cascade_2.detectMultiScale(input_image,1.2,2,cv2.CASCADE_SCALE_IMAGE,(40, 60))
        return faces
    '''



if __name__ == '__main__':
    rospy.init_node('kaka_base_face_detection', anonymous=True)
    kaka_base_face_detection = KakaBaseFaceDetection()
    kaka_base_face_detection.Run()
