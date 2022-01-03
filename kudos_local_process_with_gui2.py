"#!/usr/bin/env python"
# 라이브러리 임포트
import rospy
from sensor_msgs.msg import CompressedImage                          #이미지는 컴프레스 이미지 사용 (메세지 형태 선언안해도 될수있게 import)
from std_msgs.msg import Bool as rosBool                             # 사용 안함
from darknetb.msg import kudos_vision_local_sensor_data as kvlsd     # 행동을 멈추고 제어코드에 로봇의 정확한 위치를 요구 함
from darknetb.msg import kudos_vision_head_pub as kvhp               # 제어코드한테 머리의 각도 정보를 알려주는 코드    
from darknetb.msg import kudos_vision_op3_local_mode as kvolm        # 제어코드로부터 고개를 돌려서 로컬라이제이션 모드로 들어가라고 전달 받는 메세지. 
from PyQt5.QtWidgets import QApplication                             # GUI를 띄우기 위한 파이큐티5를 import
import bh_function.image_filter as MY_IMAGE_FILTER                   # 사용 안함
import bh_function.local_process_gui as bh_GUI                       # local_process_gui를 import ( local_process_gui는 라인을 검출하는 gui )
import numpy as np                                                   # 넘파이 import (numpy는 파이썬에서 행렬 모듈)
import sys    # sys는 경로를 추가해주고 더해주는 모듈
### 밑의 line 4개는 따로 설치하는 openCV를 사용하기 위한 코드 // 경로 추가해줄때 ros/ kinetic인지 noetic인지 버전 구분해줄 것. (20.04 18.04 16.04 각기 다름)
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')          # ROS에서 주어지는 파이썬 패키지는 안쓰니까 경로를 지워줌.
except Exception as e:
    pass


import cv2           # openCV import
from numpy.core.numeric import empty_like  # 넘파이에서 빈 행렬 만드는 모듈
import random                                     # 랜덤 모듈
from PIL import Image                          # 이미지 저장, 관리해주는 라이브러리 (몇몇기능은 openCV보다 나음)
from tqdm import tqdm                            # 반복문을 그래프로 시각화해줌
import math                                    # 수학 계산 라이브러리 
import time                                   #  sleep 같은 기능 쓰기 위한 time 라이브러리 
from multiprocessing import Process, Queue        # 코드들을 따로 돌아가게 해야돼서 import 해줌.
import multiprocessing as mp                    # 코드들을 따로 돌아가게 해야돼서 import 해줌.

save_picture_path = "./자료/"                 # 사진을 저장할 폴더

# hsv범위  ( Hue Saturation Value : RGB처럼 색을 나타내는 개념. )
mask_minimum_condition = np.array([0, 0, 235])            # HSV의 최소범위
mask_maximum_condition = np.array([180, 10, 255])         # HSV의 최대범위.
# 데이터에서 선을 추출하기 전, 변환할 사이즈와 앞으로 밀어낼 사이즈, ROI사이즈를 결정(픽셀단위)
pre_processing_size = 450              #GUI의 처리 전 변환할 사이즈. (라인 피팅시 너무 작거나 크게 보이면 이걸 변환해 주면된다.) 
                                       # 몬테 카를로 알고리즘에서의 사이즈는 900X600인데 욜로는 416X416이라 크기를 적절히 맞춰줘야함. 그렇지 않으면 사이즈가 안맞아 라인피팅 불가능. 
roi_size = {"x_size":350, "y_size":450, "pre_x":100}  # y_size는 문제가안됨. 그러나 x_size와 pre_x는 
roi_push_move = 120            # push_move는 위의 roi 사이즈 변환을 했을떄 라인이 좀 더 실물보다 가까이 있는것처럼 보이는 오류를 offset을 둬서 x값을 조절해주는 변수임.
# 팽창과 침식 정도를 결정하는 변수이다.
dilate_power = 6 #팽창강도   
erode_power = 5 #침식강도

# 대표 픽셀을 추출할 때 기준 거리 범위

# Localization 알고리즘에 20~30개의 점을 일정하게 보내기 위해, 점과 점 사이의 거리를 5~60으로 유동적인 범위를 설정.
standard_pixel_max_distance = 60           # 추출한 점과 점 사이의 거리가 최대 60까지 허용.
standard_pixel_min_distance = 5            # 추출한 점과 점 사이의 거리가 최소 5부터 허용.
standard_pixel_distance_unit = 5           # 검사 감소 거리 => 예상보다 너무 많거나 적은 추출 점이 발생한다면 '검사 감소 거리' 만큼 Distance를 증가/감소 시켜서 조절.(너무 크거나 작으면 안됨)
standard_pixel_distance = 0
# 추출되는 좌표 개수는 22~23개여야 한다. standard_pixel_distance_unit를 통해 22~23개 사이로 수렴 시키기 위해 gui에서 변수를 조정해야 함.
standard_pixel_recommend_min_num = 22  # 추출되는 최소 좌표 개슈
standard_pixel_recommend_max_num = 23  # 추출되는 최대 좌표 개수
#몬테카를로 알고리즘에 보내는 최소 최대 조건.
standard_pixel_send_min_num = 15  # 추출 좌표가 최소 15개는 넘어야 몬테카를로 알고리즘에 보낼 것.
standard_pixel_send_max_num = 30  # 추출 죄표가 최대 30개 미만이여야 몬테카를로 알고리즘에 보낼 것. (너무 많으면 피팅 잘 안됨.)
#위으 두 조건 (15~30)이 충족되어야 밑에 5초 카운트가 들어감.
standard_pixel_send_wait_count = 5      #로봇 머리 각도가 점점 천천히 하강돼서 바닥을 보기때문에 머리가 움직일 시간을 기다려주는거임.
standard_pixel_im_ready_count = 0 #유동 변수로 준비가 다 되면 하나씩 늘어난다.
standard_pixel_limit_num = 100 #몬테 카를로 알고리즘에 전송할떄 배열로 전송하는데 100은 배열 크기로 크게 의미 없는 숫자임..
optim_resize_const = 3 #해당 배수 만큼 이미지를 리사이즈 한 뒤 거리 필터를 통과시켜 최적화를 한다. ( 1 / 3으로 조절-> 추출 좌표의 절대 개수도 그만큼 감소)

#잡음 제거를 위한 가우시안 필터 강도 설정 (이미지를 부드럽게 만듦)
guassian_filter_size = 1

# 스카이 아이뷰 형 변환을 위한 사이즈 설정
empty_size_mul = 5      #로봇이 거의 바닥을 보고 있다면 empty_size_mul을 작게 잡고 반대라면 크게 잡을 것.
cp_size_left_loc = 2    
cp_size_right_loc = 3          # right_loc - left_loc은 이미지 상대적 비율

#로봇 고개 각도 설정 (시뮬레이터 속에서는 의미없고 로봇 연결했을때 의미있음)
robot_desire_tilt = 45


#경기장의 hsv범위
field_minimum_condition = np.array([20, 230, 20])
field_maximum_condition = np.array([40, 250, 40])
addw
#경기장 컨투어 팽창 침식 강도 (경기장 공백공간의 노이즈 제거)
field_contour_dilate_p = 20
field_contour_erode_p = 20

#op3_local_mode 여부
op3_local_mode = False       #정확한 위치를 알아야 될때 True로 바꿈. 이건 gui로 건드리는게 아니라 로봇이 조절하는 변수임.
start_point_x = 0
start_point_y = 0
start_point_orien = 0
start_point_xy_distribution = 0                  #확률 분포의 시작 범위 :파티클 회전 분산
start_point_orien_distribution = 0               #확률 분포의 시작 범위 : 파티클 회전 분산
start_point_diff_limit_wslow_wfast = 1.0         # 실제로 로봇이 보고있는 line과 추정된 방향으로 보는 로봇의 시야에서의 line의 차이. : 모드 변경 제한 변수 
mcl2_particle_num = 100                          # 흩뿌릴 추정 모델의 횟수
#위의 변수들은 상당히 어려운 개념이므로 몬테카를로 알고리즘의 이해가 필요

#gui와 데이터를 공유할 파라미터 메시지의 폼 형성 ( 각 프로세스간  통신을 위한 Dictionory ) 
gui_param_message_form = {
    "mask_minimum_condition": [0,0,0],
    "mask_maximum_condition": [0,0,0],
    "pre_processing_size": 0,
    "roi_size": {"x_size":0, "y_size":0, "pre_x":0},
    "roi_push_move": 0,
    "dilate_power": 0,
    "erode_power": 0,
    "guassian_filter_size": 0,
    "empty_size_mul": 0,
    "cp_size_left_loc":0,
    "cp_size_right_loc":0,
    "tilt_degree":0,
    "field_minimum_condition": [0,0,0],
    "field_maximum_condition": [0,0,0],
    "field_contour_dilate_p":0,
    "field_contour_erode_p":0,
    "standard_pixel_max_distance":0,
    "standard_pixel_min_distance":0,
    "standard_pixel_distance_unit":0,
    "standard_pixel_send_min_num":0,
    "standard_pixel_send_max_num":0,
    "standard_pixel_send_wait_count":0,
    "start_point_xy_distribution" :0,
    "start_point_orien_distribution":0,
    "start_point_diff_limit_wslow_wfast":0}

gui_param_image_form = {
    "yolo_processed_img":0,
    "hsv_img":0,
    "bin_line_img":0,
    "bin_field_img":0,
    "resize_field_img":0,
    "erode_dilate_field":0,
    "resize_img":0,
    "erode_dilate_line":0,
    "distance_line":0,
    "op3_local_mode":False,
    "num_of_line_point":0}


class priROS():
    def __init__(self):
        pass
   
    # talker는 몬테 카를로 로컬라이제이션 알고리즘에 좌표들을 보내는 함수
    def talker(self, message_form):
        pub = rospy.Publisher('kudos_vision_local_sensor_data', kvlsd, queue_size=1)
        message = kvlsd()
        message.debug_num = message_form['debug_num']                #메세지가 잘 받아지는지 아닌지에 대한 테스트를 위한 용도
        message.sensor_data_x = message_form['sensor_data_x']        # 검출된 라인 좌표들의 x값만 모아 sensor_data_x에 담아둔다.
        message.sensor_data_y = message_form['sensor_data_y']        # 검출된 라인 좌표들의 y값만 모아 sensor_data_y에 담아둔다.
        message.op3_local_mode = message_form['op3_local_mode']      # 로컬라이제이션 모드로 들어간다는 것을 몬테카를로 알고리즘에 알리기 위한 메세지
        message.xy_distribution = message_form["xy_distribution"]    # xy에 대한 분산 정보를 mcl에 전달해야 위치 추정 파티클(mcl에서 파란색 추정 입자들)이 얼마나 분포할지 결정할 수 있음.
        message.orien_distribution = message_form["orien_distribution"] # 파란색 파티클 분산들이 각도를 얼마나 할지 mcl에 보내줘야 파티클들이 각도를 얼마나 왔다갔다 할 지 결정 가능.
        message.diff_limit_wslow_wfast = message_form["diff_limit_wslow_wfast"] #모드 변경 제한 변수 / 모드 변경 제한 변수를 넘어갈정도로 도저히 위치를 못찾겠다 싶으면 푸른 파티클이 경기장 전체에 쫙 퍼져서 위치를 찾음. 
        message.start_point_x = message_form['start_point_x'] # 로봇이 파티클이 퍼져나가면서 위치를 추정할때 어디 점을 중심으로 퍼저나갈 것인지에 대한 x좌표를 오도메트리에서 받은 x를 씀.
        message.start_point_y = message_form['start_point_y'] # 그러므로 y좌표도 마찬가지로 오도메트리에서 받은 y좌표를 출발점으로 스타팅 포인트 결정.
        message.start_point_orien = message_form['start_point_orien'] # 오도메트리로 받은 각도가 스타팅 포인트에 작용.
        pub.publish(message) # mcl에 전송.
   
    # mcl에 보내는 머리의 각도
    def talker_head(self, desire_tilt, point_count):
        pub = rospy.Publisher('kudos_vision_head_pub', kvhp, queue_size=1)
        message = kvhp()
        desire_tilt = desire_tilt*math.pi/180  # 라디안 값으로 변환
        desire_tilt = -desire_tilt             # 고개를 숙여야 함으로 -부호 붙여줌.
        desire_pan = 0                         # 머리의 수평은 항상 정면이므로 0
        message.pan = desire_pan             
        message.tilt = desire_tilt
        message.point_count = point_count     # gui에 잡히는 좌표들 개수 ( .... 이면 4개 이런식으로 점들의 좌표개수 카운트. 잡히는 개수 없으면 되돌아야됨.)
        pub.publish(message)  

# 유용한 함수들 모음.
class useful_function():
    def __init__(self):
        self.pts = np.zeros((4, 2), dtype=np.float32)
        self.pts1 = 0
        self.pts2 = 0
      
    def get_distance_from_two_points(self, point1, point2):
        # 포인트의 형식은 리스트[x좌표, y좌표]
        distance = math.sqrt(((point1[0] - point2[0]) ** 2) + ((point1[1] - point2[1]) ** 2)) # 두 점사이의 거리를 피타고라스 법칙을 이용해 구해줌.
        return distance

    def save_numpy_file(self, append_name, img):
        im = Image.fromarray(img.astype('uint8'), 'RGB') #이미지를 행렬로써 다루고 그 중 원하는 이미지를 jpg로 저장할 수있음.
        im.save(save_picture_path + append_name + '.jpg')
    
    #하늘에서 본 스카이 뷰로 변환해주는 함수    ///  top_view_npArr은 kudos_vision 시뮬레이터의 오른쪽 위 (로봇이 보는 시야) 이미지를 의미. 
    def perspective(self, top_view_npArr, pre_processing_size):          # pre_processing_size : 변환할 이미지 사이즈 ( 원본 - > pre_processing_size )
        img_shape = np.shape(top_view_npArr)  # np.shape로 이미지의 사이즈를 알아내서 img_shape에 저장
        view_npArr = top_view_npArr.copy()    # view_npArr에 top_view_npArr을 복사하여 저장. (백업)
        top_view_npArr = np.zeros((img_shape[0], int(img_shape[1]*empty_size_mul), 3), dtype=np.uint8) # top_view_npArr에 hsv값이 0으로 채워진 이미지로 바꾸겠다. np.zeros(높이,가로,채널)
        top_view_npArr[: ,int(img_shape[1]*cp_size_left_loc):int(img_shape[1]*cp_size_left_loc)+np.shape(view_npArr)[1], :] = view_npArr # 0으로만 채워진 top_view_npArr에 압축 과정을 진행함.
        topLeft = [int(img_shape[0]*cp_size_left_loc),0]
        bottomRight = [int(img_shape[0]*empty_size_mul), img_shape[0]]
        topRight = [int(img_shape[0]*cp_size_right_loc),0]
        bottomLeft = [0, img_shape[0]]
        self.pts1 = np.float32([topLeft, topRight, bottomRight, bottomLeft]) # 함수를 float 32형으로 변환. int -> float 32  // pts1은 사다리꼴 좌표
        w1 = abs(bottomRight[0] - bottomLeft[0])
        w2 = abs(topRight[0] - topLeft[0])
        h1 = abs(topRight[1] - bottomRight[1])
        h2 = abs(topLeft[1] - bottomLeft[1])
        width = max([w1, w2])  # 두 좌우 거리간의 최대값이 서류의 폭
        height = max([h1, h2])  # 두 상하 거리간의 최대값이 서류의 높이
        self.pts2 = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])       # pts2는 직사각형 좌표
        mtrx = cv2.getPerspectiveTransform(self.pts1, self.pts2) # 탑뷰로 변환
        result = cv2.warpPerspective(top_view_npArr, mtrx, (int(width), int(height))) # warp로 직사각형 이미지를 mtrx라는 정보를 가지고 변환시켜줌,
        result = cv2.resize(result, dsize=(pre_processing_size, pre_processing_size), interpolation=cv2.INTER_AREA) # pre_processing_size로 리사이즈
        result = np.swapaxes(result, 0, 1) # x축과 y축을 바꿔 90도를 회전시켜줌 
        result = np.flip(result) # flip을 넣어 반대로 돌아간 이미지 조정
        result = cv2.GaussianBlur(result, (guassian_filter_size*2+1,guassian_filter_size*2+1), 0) # 이미지를 부드럽게 만들어주는 '블러'를 씌운다.

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
    
    def get_profit_point(self, optim_resize_masked_top_view_npArr, indexing_masked, standard_pixel_distance, standard_pixel_recommend_max_num, optim_resize_const):
        standard_pixel_distance /= optim_resize_const
        circle_img = np.zeros_like(optim_resize_masked_top_view_npArr, np.uint8)
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
            if standard_pixel_recommend_max_num < len(point_list):
                break

        for index, point in enumerate(point_list):
            point[0] *= optim_resize_const
            point[1] *= optim_resize_const
            point_list[index] = point 
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

def run_gui(q, paramq, gui_param_message_form, gui_param_image_form):
    app = QApplication(sys.argv)
    mywindow = bh_GUI.App(q, paramq, gui_param_message_form, gui_param_image_form)
    mywindow.show()
    app.exec_()

def when_receive_yolo_image(ros_data, args):
    #start = time.time()
    #파라미터 초기화
    priROS = args[0]
    useful_function = args[1]
    q = args[2]
    paramq = args[3]
    # hsv범위
    global mask_minimum_condition
    global mask_maximum_condition
    global pre_processing_size
    global roi_size
    global roi_push_move
    global dilate_power
    global erode_power
    global standard_pixel_max_distance
    global standard_pixel_min_distance
    global standard_pixel_distance_unit
    global standard_pixel_distance
    global standard_pixel_recommend_max_num
    global standard_pixel_recommend_min_num
    global standard_pixel_send_min_num
    global standard_pixel_send_max_num
    global standard_pixel_im_ready_count
    global standard_pixel_send_wait_count
    global standard_pixel_limit_num
    global optim_resize_const
    global guassian_filter_size

    global empty_size_mul
    global cp_size_left_loc
    global cp_size_right_loc

    global robot_desire_tilt

    global field_minimum_condition
    global field_maximum_condition

    global field_contour_dilate_p
    global field_contour_erode_p

    global op3_local_mode
    global start_point_x
    global start_point_y
    global start_point_orien
    global start_point_xy_distribution
    global start_point_orien_distribution
    global start_point_diff_limit_wslow_wfast
    global mcl2_particle_num

    mcl_message = {"debug_num":1,
                    "sensor_data_x":[],
                    "sensor_data_y":[],
                    "op3_local_mode": False,
                    "xy_distribution":0,
                    "orien_distribution":0,
                    "diff_limit_wslow_wfast":0,
                    "start_point_x":0,
                    "start_point_y":0,
                    "start_point_orien":0}
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
    gui_param_image["num_of_line_point"] = 0

    # 여기부턴 이미지 변환
    np_arr = np.fromstring(ros_data.data, np.uint8)
    view_npArr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    top_view_npArr = useful_function.perspective(view_npArr, pre_processing_size)
    gui_param_image["yolo_processed_img"] = top_view_npArr
    contour_img, gui_param_image = useful_function.field_image_mask(top_view_npArr, field_minimum_condition, field_maximum_condition, roi_size, field_contour_dilate_p, field_contour_erode_p, gui_param_image)
    masked_top_view_npArr, gui_param_image = useful_function.Image_mask(top_view_npArr, contour_img, mask_minimum_condition, mask_maximum_condition, roi_size, dilate_power, erode_power, gui_param_image)
    
    optim_resize_masked_top_view_npArr = cv2.resize(masked_top_view_npArr, 
                                                    dsize = (round(np.shape(masked_top_view_npArr)[1]/optim_resize_const), round(np.shape(masked_top_view_npArr)[0]/optim_resize_const)),
                                                    interpolation=cv2.INTER_AREA)
    indexing_masked = np.where(optim_resize_masked_top_view_npArr>254)

    point_list = []

    if standard_pixel_max_distance >= standard_pixel_min_distance:
        if standard_pixel_distance > standard_pixel_max_distance:
            standard_pixel_distance = standard_pixel_max_distance
        elif standard_pixel_distance < standard_pixel_min_distance:
            standard_pixel_distance = standard_pixel_min_distance
        point_list = useful_function.get_profit_point(optim_resize_masked_top_view_npArr, indexing_masked, standard_pixel_distance, standard_pixel_recommend_max_num, optim_resize_const)
        if len(point_list)  < standard_pixel_recommend_min_num:
            standard_pixel_distance -= standard_pixel_distance_unit
        elif len(point_list) > standard_pixel_recommend_max_num:
            standard_pixel_distance += standard_pixel_distance_unit
    
    gui_param_image["num_of_line_point"] = len(point_list)
    circle_img = np.zeros_like(masked_top_view_npArr, np.uint8)
    for point in point_list:
        cv2.circle(circle_img, (point[1], point[0]), 5, 255)
    gui_param_image["distance_line"] = circle_img
    if len(point_list) < standard_pixel_limit_num:
        for point in point_list:
            mcl_message["sensor_data_y"].append(point[0]-(pre_processing_size//2))
            mcl_message["sensor_data_x"].append(point[1]+roi_push_move)
        for i in range(standard_pixel_limit_num-len(point_list)):
            mcl_message["sensor_data_x"].append(-100)
            mcl_message["sensor_data_y"].append(-100)
        mcl_message["op3_local_mode"] = op3_local_mode
        mcl_message["start_point_x"] = start_point_x
        mcl_message["start_point_y"] = start_point_y
        mcl_message["start_point_orien"] = start_point_orien
        mcl_message["xy_distribution"] = start_point_xy_distribution
        mcl_message["orien_distribution"] = start_point_orien_distribution
        mcl_message["diff_limit_wslow_wfast"] = start_point_diff_limit_wslow_wfast

        if len(point_list) > standard_pixel_send_min_num and len(point_list) < standard_pixel_send_max_num and op3_local_mode is True:
            if standard_pixel_send_wait_count < standard_pixel_im_ready_count:
                mcl_message["op3_local_mode"] = op3_local_mode
                priROS.talker(mcl_message)
            else:
                standard_pixel_im_ready_count += 1
        elif op3_local_mode is False:
            standard_pixel_im_ready_count = 0

    priROS.talker_head(robot_desire_tilt, len(point_list))

    # 큐에 그냥 put을 하면 데드락이 걸리기 때문에, put_nowait를 써서 gui가 별개로 돌아가게 만든다
    gui_param_image["op3_local_mode"] = op3_local_mode
    try:
        q.put_nowait(gui_param_image)
    except Exception as e:
        pass

    try:
        param_dic = paramq.get_nowait()
        mask_minimum_condition = param_dic["mask_minimum_condition"]
        mask_maximum_condition = param_dic["mask_maximum_condition"]
        pre_processing_size = param_dic["pre_processing_size"]
        roi_size = param_dic["roi_size"]
        roi_push_move = param_dic["roi_push_move"]
        dilate_power = param_dic["dilate_power"]
        erode_power = param_dic["erode_power"]
        standard_pixel_max_distance = param_dic["standard_pixel_max_distance"]
        standard_pixel_min_distance = param_dic["standard_pixel_min_distance"]
        standard_pixel_distance_unit = param_dic["standard_pixel_distance_unit"]
        standard_pixel_send_min_num = param_dic["standard_pixel_send_min_num"]
        standard_pixel_send_max_num = param_dic["standard_pixel_send_max_num"]
        standard_pixel_send_wait_count = param_dic["standard_pixel_send_wait_count"]
        guassian_filter_size = param_dic["guassian_filter_size"]
        if (param_dic["cp_size_left_loc"] + param_dic["cp_size_right_loc"]) <= param_dic["empty_size_mul"]: 
            empty_size_mul = param_dic["empty_size_mul"]
            cp_size_left_loc = param_dic["cp_size_left_loc"]
            cp_size_right_loc = param_dic["cp_size_right_loc"]
        robot_desire_tilt = param_dic["tilt_degree"]
        field_minimum_condition = param_dic["field_minimum_condition"]
        field_maximum_condition = param_dic["field_maximum_condition"]
        field_contour_dilate_p = param_dic["field_contour_dilate_p"]
        field_contour_erode_p = param_dic["field_contour_erode_p"]
        start_point_xy_distribution = param_dic["start_point_xy_distribution"]
        start_point_orien_distribution = param_dic["start_point_orien_distribution"]
        start_point_diff_limit_wslow_wfast = param_dic["start_point_diff_limit_wslow_wfast"]
        
        mask_minimum_condition = np.array(mask_minimum_condition)
        mask_maximum_condition = np.array(mask_maximum_condition)
        field_minimum_condition = np.array(field_minimum_condition)
        field_maximum_condition = np.array(field_maximum_condition)
    except Exception as e:
        pass

    #end = time.time()
    #print("코드 총 실행 시간:{:.5f}, 감지된 포인트당 수행시간:{:.5f}".format(start-end, (start-end)/len(point_list)))

def when_receive_op3_local_msg(ros_data, args):
    priROS = args[0]
    global op3_local_mode
    global start_point_x
    global start_point_y
    global start_point_orien
    op3_local_mode = ros_data.op3_local_mode
    start_point_x = ros_data.start_point_x
    start_point_y = ros_data.start_point_y
    start_point_orien = ros_data.start_point_orien


if __name__=='__main__':
    priROS = priROS() # 상단의 priROS 클래스 초기화
    useful_function = useful_function() #유용한 함수들을 class로 묶은 것이 useful_function. 메인문에서 초기화.
    q = Queue(1)
    paramq = Queue(1)
    p = Process(name="producer", target=run_gui, args=(q, paramq, gui_param_message_form, gui_param_image_form), daemon=True)
    p.start()
    rospy.init_node('kudos_vision_local_process', anonymous = False)                          #노드 초기화
    rospy.Subscriber("/output/image_raw/compressed", CompressedImage, when_receive_yolo_image,(priROS, useful_function, q, paramq), queue_size=1) # 이미지를 받는 subscriber
    rospy.Subscriber("kudos_vision_op3_local_mode", kvolm, when_receive_op3_local_msg,(priROS, ), queue_size=1)        # op3 메세지를 받는 subscriber 
    # 각각 메세지를 받을 때, 각각 when_receive_yolo_image,(priROS, useful_function, q, paramq)  when_receive_op3_local_msg,(priROS, )를 실행. 인자는 괄호 안의 인자들이 들어감.
    rospy.spin()
