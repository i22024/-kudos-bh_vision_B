"#!/usr/bin/env python"
IS_SIMULATOR = True
SAVE_INPUT_VIDEO = False
import rospy
import roslib
from darknetb.msg import kudos_vision_ball_position as kvbp
from sensor_msgs.msg import CompressedImage
# import tensorflow as tf
import darknet
import kudos_darknet
from multiprocessing import Process, Queue
import numpy as np
import roslib
from numpy.core.numeric import empty_like
import random
from PIL import Image
from tqdm import tqdm
import math
import time
from collections import deque
try:
    import sys
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except Exception as e:
    pass
import cv2
import multiprocessing as mp
if IS_SIMULATOR:
    import bh_function.CustomFuncionFor_mlAgent as CF
    from mlagents_envs.rpc_communicator import RpcCommunicator
    from mlagents_envs.environment import UnityEnvironment
    from mlagents_envs.base_env import ActionTuple
    from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel


# 전역 변수 설정
if IS_SIMULATOR:
    game = "vision_simulator.x86_64"
    env_path = "./Linux_soccer_simulator/" + game
    save_picture_path = "./made_data/"
    channel = EngineConfigurationChannel()
    channel.set_configuration_parameters(time_scale=1.0, target_frame_rate=60, capture_frame_rate=60)
    env = UnityEnvironment(file_name=env_path, side_channels=[channel])
    env.reset()
    behavior_names = list(env.behavior_specs)
    ConversionDataType = CF.ConversionDataType()
    AgentsHelper = CF.AgentsHelper(env, string_log=None, ConversionDataType=ConversionDataType)
    write_file_name_list_index_instead_of_correct_name = False
    list_index_for_ALL = 0
    list_index_for_ball = 2
    list_index_for_stage = 1
    list_index_for_goal1_detection = 3
    list_index_for_goal2_detection = 4
    list_index_for_goal1_range = 5
    list_index_for_goal2_range = 6
    list_index_for_top_view = 7

# yolo 세팅
yolo_processed_size = 416
capture_target = 0
default_x = -100.0
default_y = -100.0

#yolo mask 설정
#mask_img는 기본적으로 전부 0인 행렬이며, 감지된 박스에 의해 값이 오르더라도 서서히 값이 0로 간다
mask_overcoat_power = 70
mask_remove_overcoat_power = 10
mask_img = np.zeros((416, 416,3),dtype = np.uint8)
mask_remove_overcoat_img = np.zeros((416, 416,3),dtype = np.uint8)
mask_remove_overcoat_img[:,:,:] = mask_remove_overcoat_power 

class priROS():
    def __init__(self):
        rospy.init_node('kudos_vision', anonymous = False)
        self.yolo_pub = rospy.Publisher('kudos_vision_ball_position', kvbp, queue_size=1)
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size = 1)
        self.yolo_result_img_pub = rospy.Publisher("/output/image_raw2/compressed", CompressedImage, queue_size = 1)

    def yolo_talker(self, message_form):
        message = kvbp()
        message.posX = message_form['posX']
        message.posY = message_form['posY']
        message.goalposX = message_form['goalposX']
        message.goalposY = message_form['goalposY']
        message.POS_size = message_form['ball_size']
        message.POS_distance = message_form['ball_distance']
        self.yolo_pub.publish(message)
    
    def img_talker(self, image_np):
        import cv2
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.image_pub.publish(msg)

    def yolo_result_img_talker(self, image_np):
        import cv2
        print(np.shape(image_np))
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.yolo_result_img_pub.publish(msg)

class DataFormatTransfer():
    def __init__(self):
        pass
    
    def get_one_center_from_detections(self, detections, label):
        max_confidence = 0
        objectCenter = [default_x ,default_y]
        object_size = 0
        for detections_index, detection in enumerate(detections):
            #print(detection)#(label, confidence, (left, top, right, bottom))
            if detection[0] == label:
                if float(detection[1])>max_confidence:
                    max_confidence = float(detection[1])
                    bbox = detection[2]
                    objectCenter = [bbox[0], bbox[1]]
                    object_size = (bbox[2]+bbox[3])/2
        return objectCenter, object_size

    def get_mean_center_from_detections(self, detections, label):
        objectCenter = [default_x ,default_y]
        object_width_list = []
        object_height_list = []
        detect_flag = False
        for detections_index, detection in enumerate(detections):
            #print(detection)#(label, confidence, (left, top, right, bottom))
            if detection[0] == label:
                bbox = detection[2]
                object_width_list.append(bbox[0])
                object_height_list.append(bbox[1])
                detect_flag = True
        if detect_flag == True:
            objectCenter = [np.mean(object_width_list), np.mean(object_height_list)]

        return objectCenter, 0

    def fill_black_on_detect_box(self, detections, img):
        global mask_img
        global mask_remove_overcoat_img
        mask_overcoat_img = np.zeros((yolo_processed_size, yolo_processed_size,3), dtype = np.uint8)
        for detections_index, detection in enumerate(detections):
            bbox = detection[2]
            start_w = round(bbox[0]-bbox[2]/2)
            start_h = round(bbox[1]-bbox[3]/2)
            end_w = round(bbox[0]+bbox[2]/2)
            end_h = round(bbox[1]+bbox[3]/2)
            mask_overcoat_img[start_h:end_h, start_w:end_w,:] = np.array([mask_overcoat_power,mask_overcoat_power,mask_overcoat_power])
        
        mask_img = cv2.add(mask_img, mask_overcoat_img)
        mask_img = cv2.subtract(mask_img, mask_remove_overcoat_img)
        img = cv2.subtract(img, mask_img)
        return img

    def mapping_point_to_float_shape(self, npArr, objectCenter, objectSize):
        if objectCenter != [default_x, default_y]:
            im_width_size = np.shape(npArr)[0]
            im_hight_size = np.shape(npArr)[1]
            objectSize = objectSize/((im_width_size+im_hight_size)/2)
            objectCenter[0] = objectCenter[0]/im_width_size
            objectCenter[1] = objectCenter[1]/im_hight_size
            objectCenter[0] = (objectCenter[0] - 0.5)
            objectCenter[1] = (objectCenter[1] - 0.5)

        return objectCenter, objectSize

    def get_distance_from_ball_size(self, ball_size):
        distance = -1
        if ball_size == 0:
            distance = -1
        elif ball_size>0.06:
            distance = 5.8729*((ball_size)**(-1.342))
        else:
            distance = 200

        return distance

if __name__=='__main__':
    darknet_config_args = kudos_darknet.parser()
    kudos_darknet.check_arguments_errors(darknet_config_args)
    darknet_network, darknet_class_names, darknet_class_colors, darknet_width, darknet_height = kudos_darknet.Initialize_darknet(darknet_config_args)
    if not IS_SIMULATOR:
        cap = cv2.VideoCapture(capture_target)
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    
    if SAVE_INPUT_VIDEO:
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        out = cv2.VideoWriter('save.avi', fourcc, 25.0,(yolo_processed_size, yolo_processed_size))
    priROS = priROS()
    DataFormatTransfer = DataFormatTransfer()

    while True:
        if not IS_SIMULATOR:
            ret, origin_img = cap.read()
            input_img_size = np.shape(origin_img)
            crop_size = min(input_img_size[0], input_img_size[1])
            start_y = round((input_img_size[0]-crop_size)/2)
            start_x = round((input_img_size[1]-crop_size)/2)
            origin_img = origin_img[start_y:start_y+crop_size, start_x:start_x+crop_size, :]
            origin_img = cv2.resize(origin_img, dsize=(yolo_processed_size, yolo_processed_size), interpolation=cv2.INTER_AREA)
        elif IS_SIMULATOR:
            behavior_name = behavior_names[0]
            decision_steps, terminal_steps = env.get_steps(behavior_name)
            vec_observation, vis_observation_list, done = AgentsHelper.getObservation(behavior_name)
            ret = True
            origin_img = vis_observation_list[list_index_for_ALL]
            origin_img = cv2.cvtColor(origin_img, cv2.COLOR_BGR2RGB)

        if SAVE_INPUT_VIDEO == True:
            out.write(origin_img)
        frame, detections = kudos_darknet.getResults_with_darknet(ret, origin_img, darknet_width, darknet_height, darknet_network, darknet_class_names, darknet_class_colors,darknet_config_args)
        ballCenter = [-100.0, -100.0]
        goalCenter = [-100.0, -100.0]
        ball_size = 0
        if np.any(frame) != False:
            #cv2.imshow("showIMG", frame)
            priROS.yolo_result_img_talker(frame)
            ballCenter, ball_size = DataFormatTransfer.get_one_center_from_detections(detections, label='ball')
            ballCenter,ball_size = DataFormatTransfer.mapping_point_to_float_shape(frame, ballCenter, ball_size)
            goalCenter,_ = DataFormatTransfer.get_mean_center_from_detections(detections, label='goal')
            goalCenter,_ = DataFormatTransfer.mapping_point_to_float_shape(frame, goalCenter, 0)
        posX = ballCenter[0]
        posY = ballCenter[1]
        ball_distance = DataFormatTransfer.get_distance_from_ball_size(ball_size)
        goalposX = goalCenter[0]
        goalposY = goalCenter[1]
        vision_message = {
            'posX':posX,
            'posY':posY,
            'goalposX':goalposX,
            'goalposY':goalposY,
            'ball_size':ball_size,
            'ball_distance':ball_distance,
            'desire_pan':0.0,
            'desire_tilt':0.0}
        #message_form["desire_pan"], message_form["desite_tilt"] = kudos_tracker.tracking_ball(message_form)
        priROS.yolo_talker(vision_message)
        black_on_detect_box_img = DataFormatTransfer.fill_black_on_detect_box(detections, origin_img)
        priROS.img_talker(black_on_detect_box_img)
       

        if IS_SIMULATOR:
            action = [3, 4, 0, 40]
            actionTuple = ConversionDataType.ConvertList2DiscreteAction(action, behavior_name)
            env.set_actions(behavior_name, actionTuple)
            env.step()

    cap.release
    cv2.destroyAllWindows()
