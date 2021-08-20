"#!/usr/bin/env python"
# 시스템 결정 변수
IS_SIMULATOR = False

# 라이브러리 임포트
import rospy
from sensor_msgs.msg import CompressedImage
from PyQt5.QtWidgets import QApplication
import bh_function.image_filter as MY_IMAGE_FILTER
import bh_function.local_process_gui as bh_GUI
import numpy as np
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except Exception as e:
    pass
import cv2
from darknetb.msg import mcl2_sensor_data
from numpy.core.numeric import empty_like
import random
from PIL import Image
from tqdm import tqdm
import math
import time
from multiprocessing import Process, Queue
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

# hsv범위
mask_minimum_condition = np.array([0, 0, 235])
mask_maximum_condition = np.array([180, 10, 255])
# 데이터에서 선을 추출하기 전, 변환할 사이즈와 앞으로 밀어낼 사이즈, ROI사이즈를 결정(픽셀단위)
pre_processing_size = 450
roi_size = {"x_size":350, "y_size":450, "pre_x":100}
roi_push_move = 120
# 팽창과 침식 정도를 결정하는 변수이다.
dilate_power = 6 #팽창강도
erode_power = 5 #침식강도
# 대표 픽셀을 추출할 때 기준 거리를 결정하는 변수이다.
standard_pixel_distance = 40
#잡음 제거를 위한 가우시안 필터 강도 설정
guassian_filter_size = 1

#gui 이미지 표시 설정
img1 = 0
img2 = 0
img3 = 0
img4 = 0
img_size = 150

#형 변환 사이즈 설정
empty_size_mul = 5
cp_size_left_loc = 2
cp_size_right_loc = 3

#로봇 고개 각도 설정
robot_desire_tilt = 1.57/2

#경기장의 hsv범위
field_minimum_condition = np.array([20, 230, 20])
field_maximum_condition = np.array([40, 250, 40])

#경기장 컨투어 팽창 침식 강도
field_contour_dilate_p = 20
field_contour_erode_p = 20

#gui와 데이터를 공유할 파라미터 메시지의 폼 형성
gui_param_message_form = {
    "mask_minimum_condition": [0,0,0],
    "mask_maximum_condition": [0,0,0],
    "pre_processing_size": 0,
    "roi_size": {"x_size":0, "y_size":0, "pre_x":0},
    "roi_push_move": 0,
    "dilate_power": 0,
    "erode_power": 0,
    "pixel_distance": 0,
    "guassian_filter_size": 0,
    "empty_size_mul": 0,
    "cp_size_left_loc":0,
    "cp_size_right_loc":0,
    "tilt_degree":0,
    "field_minimum_condition": [0,0,0],
    "field_maximum_condition": [0,0,0],
    "field_contour_dilate_p":0,
    "field_contour_erode_p":0}

gui_param_image_form = {
    "yolo_processed_img":0,
    "hsv_img":0,
    "bin_line_img":0,
    "bin_field_img":0,
    "resize_field_img":0,
    "erode_dilate_field":0,
    "resize_img":0,
    "erode_dilate_line":0,
    "distance_line":0}

class priROS():
    def __init__(self):
        rospy.init_node('soccer_simu_sensor_processor', anonymous = False)

    def talker(self, message_form):
        pub = rospy.Publisher('soccer_simu_sensor_data', mcl2_sensor_data, queue_size=1)
        message = mcl2_sensor_data()
        message.debug_num = message_form['debug_num']
        message.sensor_data_x = message_form['sensor_data_x']
        message.sensor_data_y = message_form['sensor_data_y']
        pub.publish(message)
    
    def listner(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return image_np

class useful_function():
    def __init__(self):
        self.pts = np.zeros((4, 2), dtype=np.float32)
        self.pts1 = 0
        self.pts2 = 0

    def get_distance_from_two_points(self, point1, point2):
        # 포인트의 형식은 리스트[x좌표, y좌표]
        distance = math.sqrt(((point1[0] - point2[0]) ** 2) + ((point1[1] - point2[1]) ** 2))
        return distance

    def save_numpy_file(self, append_name, img):
        im = Image.fromarray(img.astype('uint8'), 'RGB')
        im.save(save_picture_path + append_name + '.jpg')

    def perspective(self, top_view_npArr, pre_processing_size):
        img_shape = np.shape(top_view_npArr)
        view_npArr = top_view_npArr.copy()
        top_view_npArr = np.zeros((img_shape[0], int(img_shape[1]*empty_size_mul), 3), dtype=np.uint8)
        top_view_npArr[: ,int(img_shape[1]*cp_size_left_loc):int(img_shape[1]*cp_size_left_loc)+np.shape(view_npArr)[1], :] = view_npArr
        topLeft = [int(img_shape[0]*cp_size_left_loc),0]
        bottomRight = [int(img_shape[0]*empty_size_mul), img_shape[0]]
        topRight = [int(img_shape[0]*cp_size_right_loc),0]
        bottomLeft = [0, img_shape[0]]
        self.pts1 = np.float32([topLeft, topRight, bottomRight, bottomLeft])
        w1 = abs(bottomRight[0] - bottomLeft[0])
        w2 = abs(topRight[0] - topLeft[0])
        h1 = abs(topRight[1] - bottomRight[1])
        h2 = abs(topLeft[1] - bottomLeft[1])
        width = max([w1, w2])  # 두 좌우 거리간의 최대값이 서류의 폭
        height = max([h1, h2])  # 두 상하 거리간의 최대값이 서류의 높이
        self.pts2 = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
        mtrx = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        result = cv2.warpPerspective(top_view_npArr, mtrx, (int(width), int(height)))
        result = cv2.resize(result, dsize=(pre_processing_size, pre_processing_size), interpolation=cv2.INTER_AREA)
        result = np.swapaxes(result, 0, 1)
        result = np.flip(result)
        result = cv2.GaussianBlur(result, (guassian_filter_size*2+1,guassian_filter_size*2+1), 0)

        return result
    
    def Image_mask(self, top_view_npArr, contour_img, mask_minimum_condition, mask_maximum_condition, roi_size, dilate_power, erode_power, gui_param_image):
        hsv_top_view_npArr = cv2.cvtColor(top_view_npArr, cv2.COLOR_BGR2HSV)
        gui_param_image["hsv_img"] = hsv_top_view_npArr
        #gui_image_list.append(hsv_top_view_npArr)
        '''
        h2, s2, v2 = cv2.split(hsv_top_view_npArr)
        cv2.imshow('h', h2) # 색상 범위는 (0~180), 하얀색에서는 채도가 제대로 결정되지 않음
        cv2.imshow('s', s2) # 채도 범위는 원색의 강렬함 정도, 0~255사이의 값으로 표현된다. 하얀색에서는 원색의 강렬함 정도가 낮게 표현된
        cv2.imshow('v', v2) # 명도는 색의 밝고 어두운 정도를 표현한다. 0~255사이의 값으로 표현된다. 시뮬상에서는 하얀색과 초록색이 구분되지 않는다.
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        '''
        # 지정한 범위로 마스크를 씌워 하얀색 영역만 검출해낸다.
        masked_hsv_top_view_npArr = cv2.inRange(hsv_top_view_npArr, mask_minimum_condition, mask_maximum_condition)
        gui_param_image["bin_line_img"] = masked_hsv_top_view_npArr
        #gui_image_list.append(masked_hsv_top_view_npArr)

        #위치 검출기의 범위를 벗어나서 검출되는 선이 발생하지 않도록 ROI를 설정한다.
        masked_hsv_top_view_npArr = masked_hsv_top_view_npArr[:,roi_size["pre_x"]:roi_size["pre_x"]+roi_size["x_size"]]
        gui_param_image["resize_img"] = masked_hsv_top_view_npArr
        #gui_image_list.append(masked_hsv_top_view_npArr)

        ###
        #To Do
        #여기에 필드와 라인과의 교집합을 설정한다.
        mask_img = np.empty_like(masked_hsv_top_view_npArr)
        mask_img[:,:] = 0
        mask_img[contour_img>0] = masked_hsv_top_view_npArr[contour_img>0]
        masked_hsv_top_view_npArr = mask_img
        ###

        # 팽창을 이용하여 라인사이에 존재하는 노이즈를 없애고 침식을 이용해 라인을 얇게 형성한다.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dilate_power, dilate_power))
        masked_hsv_top_view_npArr = cv2.dilate(masked_hsv_top_view_npArr, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (erode_power, erode_power))
        masked_hsv_top_view_npArr = cv2.erode(masked_hsv_top_view_npArr, kernel)
        gui_param_image["erode_dilate_line"] = masked_hsv_top_view_npArr
        #gui_image_list.append(masked_hsv_top_view_npArr)

        return masked_hsv_top_view_npArr, gui_param_image
    
    def get_profit_point(self, masked_top_view_npArr, indexing_masked, standard_pixel_distance):
        circle_img = np.zeros_like(masked_top_view_npArr, np.uint8)
        point_list = []
        for index in range(len(indexing_masked[0])):
            tmp_point = [indexing_masked[0][index], indexing_masked[1][index]]
            if len(point_list) == 0:
                point_list.append(tmp_point)
            else:
                min_distance = 999
                for point in point_list:
                     distance = self.get_distance_from_two_points(point, tmp_point)
                     if min_distance>distance:
                         min_distance = distance
                if min_distance > standard_pixel_distance:
                    point_list.append(tmp_point)

        final_point_list = point_list.copy()
        '''
        total_point_list = []
        for point in final_point_list:
            min_distance = 9999
            point_list.remove(point)
            for target_point in point_list:
                distance = self.get_distance_from_two_points(target_point, point)
                if distance<min_distance:
                    min_distance = distance
            if min_distance<delete_noise_distance:
                total_point_list.append(point)
            point_list.append(point)
        '''
        return point_list


    def field_image_mask(self, top_view_npArr, field_minimum_condition, field_maximum_condition, roi_size, field_contour_dilate_p, field_contour_erode_p, gui_param_image):
        top_view_npArr = cv2.cvtColor(top_view_npArr, cv2.COLOR_BGR2HSV)
        contour_img = np.empty_like(top_view_npArr)
        contour_img[:,:,:] = np.array([0,0,0])
        contour_img = cv2.inRange(top_view_npArr, field_minimum_condition, field_maximum_condition)
        gui_param_image["bin_field_img"] = contour_img

        contours, _ = cv2.findContours(contour_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contour_img[:,:] = np.array(0)
        for contour in contours:
            cv2.drawContours(contour_img, [contour],0, (255, 255, 255),3)
        cv2.fillPoly(contour_img, pts=contours, color=(255,255,255))
        contour_img = contour_img[:, roi_size["pre_x"]:roi_size["pre_x"]+roi_size["x_size"]]
        gui_param_image["resize_field_img"] = contour_img

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (field_contour_dilate_p, field_contour_dilate_p))
        contour_img = cv2.dilate(contour_img, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (field_contour_erode_p, field_contour_erode_p))
        contour_img = cv2.erode(contour_img, kernel)
        gui_param_image["erode_dilate_field"] = contour_img

        return contour_img, gui_param_image

def run_gui(data_queue, paramq, gui_param_message_form, gui_param_image_form):
    app = QApplication(sys.argv)
    mywindow = bh_GUI.App(q, paramq, gui_param_message_form, gui_param_image_form)
    mywindow.show()
    app.exec_()

if __name__ == "__main__":
    priROS = priROS()
    useful_function = useful_function()
    q = Queue(1)
    paramq = Queue(1)
    p = Process(name="producer", target=run_gui, args=(q, paramq, gui_param_message_form, gui_param_image_form), daemon=True)
    p.start()
    count = 0
    while 1:
        # montecalor localization 알고리즘에 보낼 메세지 폼
        mcl_message = {
            "debug_num":1,
            "sensor_data_x":[],
            "sensor_data_y":[]}
        gui_param_image = gui_param_image_form
        gui_param_image["yolo_processed_img"] = 0
        gui_param_image["hsv_img"] = 0
        gui_param_image["bin_line_img"] = 0
        gui_param_image["bin_field_img"] = 0
        gui_param_image["resize_field_img"]=0
        gui_param_image["erode_dilate_field"]=0
        gui_param_image["resize_img"] = 0
        gui_param_image["erode_dilate_line"] = 0
        gui_param_image["distance_line"] = 0
        if IS_SIMULATOR:
            behavior_name = behavior_names[0]
            decision_steps, terminal_steps = env.get_steps(behavior_name)
            vec_observation, vis_observation_list, done = AgentsHelper.getObservation(behavior_name)
            view_npArr = vis_observation_list[list_index_for_ALL]
            view_npArr = cv2.cvtColor(view_npArr, cv2.COLOR_RGB2BGR)
        elif not IS_SIMULATOR:
            ros_data = rospy.wait_for_message("/output/image_raw/compressed", CompressedImage, timeout=None)
            view_npArr = priROS.listner(ros_data)
        top_view_npArr = useful_function.perspective(view_npArr, pre_processing_size)
        gui_param_image["yolo_processed_img"] = top_view_npArr

        contour_img, gui_param_image = useful_function.field_image_mask(top_view_npArr, field_minimum_condition, field_maximum_condition, roi_size, field_contour_dilate_p, field_contour_erode_p, gui_param_image)
        
        masked_top_view_npArr, gui_param_image = useful_function.Image_mask(top_view_npArr, contour_img, mask_minimum_condition, mask_maximum_condition, roi_size, dilate_power, erode_power, gui_param_image)
        indexing_masked = np.where(masked_top_view_npArr>254)
        
        point_list = useful_function.get_profit_point(masked_top_view_npArr, indexing_masked, standard_pixel_distance)
        circle_img = np.zeros_like(masked_top_view_npArr, np.uint8)
        for point in point_list:
            cv2.circle(circle_img, (point[1], point[0]), 5, 255)
        
        gui_param_image["distance_line"] = circle_img

        for point in point_list:
            mcl_message["sensor_data_y"].append(point[0]-(pre_processing_size//2))
            mcl_message["sensor_data_x"].append(point[1]+roi_push_move)
        for i in range(100-len(point_list)):
            mcl_message["sensor_data_x"].append(-100)
            mcl_message["sensor_data_y"].append(-100)

        priROS.talker(mcl_message)

        # 큐에 그냥 put을 하면 데드락이 걸리기 때문에, put_nowait를 써서 gui가 별개로 돌아가게 만든다
        try:
            q.put_nowait(gui_param_image)
        except Exception as e:
            pass

        try:
            param_dic = paramq.get_nowait()
            mask_minimum_condition = param_dic["mask_minimum_condition"]
            mask_maximum_condition = param_dic["mask_maximum_condition"]
            pre_process_size = param_dic["pre_processing_size"]
            roi_size = param_dic["roi_size"]
            roi_push_move = param_dic["roi_push_move"]
            dilate_power = param_dic["dilate_power"]
            erode_power = param_dic["erode_power"]
            standard_pixel_distance = param_dic["pixel_distance"]
            guassian_filter_size = param_dic["guassian_filter_size"]
            empty_size_mul = param_dic["empty_size_mul"]
            cp_size_left_loc = param_dic["cp_size_left_loc"]
            cp_size_right_loc = param_dic["cp_size_right_loc"]
            robot_desire_tilt = param_dic["tilt_degree"]
            field_minimum_condition = param_dic["field_minimum_condition"]
            field_maximum_condition = param_dic["field_maximum_condition"]
            field_contour_dilate_p = param_dic["field_contour_dilate_p"]
            field_contour_erode_p = param_dic["field_contour_erode_p"]
            
            mask_minimum_condition = np.array(mask_minimum_condition)
            mask_maximum_condition = np.array(mask_maximum_condition)
            field_minimum_condition = np.array(field_minimum_condition)
            field_maximum_condition = np.array(field_maximum_condition)
        except Exception as e:
            pass


        if IS_SIMULATOR:
            action = [3, 4, 0, int(robot_desire_tilt)]
            actionTuple = ConversionDataType.ConvertList2DiscreteAction(action, behavior_name)
            env.set_actions(behavior_name, actionTuple)
            env.step()
    
    if IS_SIMULATOR:
        env.close()
