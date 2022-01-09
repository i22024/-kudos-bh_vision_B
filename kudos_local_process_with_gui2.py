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
    
    # sky view 컬러 화면에서 필드는 검은색, line은 흰색으로 변환시키는 함수 ( gui에서 1번째 화면 - > 2번째 화면 )
    # 원리 HSV 최소~ 최대 범위 사이에 있다면 흰색으로 만들고, 그 범위 바깥에 있다면 검은색으로 칠해버린다.
    def Image_mask(self, top_view_npArr, contour_img, mask_minimum_condition, mask_maximum_condition, roi_size, dilate_power, erode_power, gui_param_image):
        hsv_top_view_npArr = cv2.cvtColor(top_view_npArr, cv2.COLOR_BGR2HSV) # top_view_npArr을 hsv로 변환.
        # Image_mask의 매개변수 top_view_npArr : gui 1번쨰 화면 // contour_img : hsv 범위를 조정하여 gui 아래쪽 흑백 반전한 화면에서 팽창과 침식을 이용해 line을 지우고 경기장만 잡은 이미지. gui 하단 마지막 이미지.
        # mask_minimum_condition, mask_maximum_condition : HSV 최소, 최대 범위를 저장하는 리스트 
        # roi_size는 자를 이미지의 사이즈. 딕셔너리 형이며 gui에 설정된 값으로 덮어씌워짐. // dilate_power, erode_power는 각각 팽창, 침식 강도이다. 팽창: 하얀색 영역이 팽창. 침식은 그 반대.
        # gui_param_image는 이미지를 저장하고 GUI에 쏘아주는 변수. gui_param_image_form에 리스트가 저장되어 있음.
        gui_param_image["hsv_img"] = hsv_top_view_npArr  # gui_param_image는 gui에게 이미지 파라미터들을 묶어보내는 택배박스. gui_param_image의 "hsv_img"변수에  hsv_top_view_npArr 대입.
        #gui_image_list.append(hsv_top_view_npArr)
        '''
        h2, s2, v2 = cv2.split(hsv_top_view_npArr) # 3차원 채널의 이미지를 채널별로 분류해 h2 s2 v2에 2차원으로 저장.
        # imshow는 이미지를 보여주는 것. hsv 변환이 잘 됐는지 확인하는 코드.
        cv2.imshow('h', h2) # 색상 범위는 (0~180), 하얀색에서는 채도가 제대로 결정되지 않음
        cv2.imshow('s', s2) # 채도 범위는 원색의 강렬함 정도, 0~255사이의 값으로 표현된다. 하얀색에서는 원색의 강렬함 정도가 낮게 표현된
        cv2.imshow('v', v2) # 명도는 색의 밝고 어두운 정도를 표현한다. 0~255사이의 값으로 표현된다. 시뮬상에서는 하얀색과 초록색이 구분되지 않는다.
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        '''
        #inRange를 이용하여 hsv_top_view_npArr의  mask_minimum_condition ~ mask_maximum_condition 사이에 있는, 즉 하얀색 영역만 검출하고, 나머지는 다 검은색으로 처리하여
        # 변수 masked_hsv_top_view_npArr에 저장한다. 이 이미지는 gui에서 상단 2번째 흑백 이미지다. 
        masked_hsv_top_view_npArr = cv2.inRange(hsv_top_view_npArr, mask_minimum_condition, mask_maximum_condition)      
        gui_param_image["bin_line_img"] = masked_hsv_top_view_npArr # 변수  masked_hsv_top_view_npArr의 이미지를 gui_param_image의 bin_line_img에 저장한다.
        #현재 masked_hsv_top_view_npArr에는 이진 이미지(상단 2번째 흑백이미지)가 저장되어있다.
        #gui_image_list.append(masked_hsv_top_view_npArr)

        #위치 검출기의 범위를 벗어나서 검출되는 선이 발생하지 않도록 ROI를 설정한다.
        masked_hsv_top_view_npArr = masked_hsv_top_view_npArr[:,roi_size["pre_x"]:roi_size["pre_x"]+roi_size["x_size"]]
        gui_param_image["resize_img"] = masked_hsv_top_view_npArr 
        #여기서 resize_img는 gui에서 3번째 화면으로써, 기존 2번째 gui 화면인 masked_hsv_top_view_npArr를 pre_x지점부터 pre_x+x_size 지점까지 잘라 gui_param_image의 resize_img변수에 저장한다.
        #gui_image_list.append(masked_hsv_top_view_npArr)

        ###
        #To Do
        #여기에 필드와 라인과의 교집합을 설정한다. 즉 gui의 3번째 이미지인 masked_hsv_top_view_npArr(=resize_img)와 contour_img의 교집합이
        # gui의 4번째 화면인 mask_img가 된다.
        mask_img = np.empty_like(masked_hsv_top_view_npArr) # masked_hsv_top_view_npArr와 같은 모양과 크기인데, hsv값이 전부 0으로 채워진 이미지가  mask_img에 저장된다.
        mask_img[:,:] = 0 # 위 코드에 의해  mask_img에 시스템에 따라 0이 채워질수도, num이 채워질수도 있는데 그냥 편의를 위해 0을 채움.
        mask_img[contour_img>0] = masked_hsv_top_view_npArr[contour_img>0]
        masked_hsv_top_view_npArr = mask_img
        ###
        
        # 팽창 침식 알고리즘. 팽창과 침식을 하려면 getStructuringElement 함수를 써야한다.
        # 팽창을 이용하여 라인사이에 존재하는 노이즈를 없애고 침식을 이용해 라인을 얇게 형성한다.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dilate_power, dilate_power))
        masked_hsv_top_view_npArr = cv2.dilate(masked_hsv_top_view_npArr, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (erode_power, erode_power))
        masked_hsv_top_view_npArr = cv2.erode(masked_hsv_top_view_npArr, kernel)
        gui_param_image["erode_dilate_line"] = masked_hsv_top_view_npArr
        #gui_image_list.append(masked_hsv_top_view_npArr)

        return masked_hsv_top_view_npArr, gui_param_image
    
    # 검출된 좌표들의 위치를 반환하는 함수.
    def get_profit_point(self, optim_resize_masked_top_view_npArr, indexing_masked, standard_pixel_distance, standard_pixel_recommend_max_num, optim_resize_const):
        # optim_resize_masked_top_view_npArr는 masked_top_view_npArr를 optim_resize_const로 나눠서 사이즈 조정한 것.
        # indexing_masked는 optim_resize_masked_top_view_npArr에서 하얀색인 부분들의 좌표들.
        #  standard_pixel_recommend_max_num은 화면이 어둡다가 확~ 밝아지면 그 순간 모든 지점을 좌표로 인식해서 연산하기 때문에 느려지고 연산이 밀리게 되고 코드가 멈추는 치명적인 에러가 발생한다.
        # 따라서 그 연산하는 좌표점의 개수를 제한하는  standard_pixel_recommend_max_num 변수를 통해 에러 통제.
        # optim_resize_const를 통해 리사이즈를 하면 추출하는 좌표 개수를 줄일 수 있기 때문에 연산이 빨라진다. 그리고 줄어든 좌표점 연산을 다시 복원하기 위해 존재하는 변수다.
        
        standard_pixel_distance /= optim_resize_const #이미지가 리사이즈 된 만큼, 좌표점들의 거리도 리사이즈 시켜주는 과정.
        circle_img = np.zeros_like(optim_resize_masked_top_view_npArr, np.uint8) #optim_resize_masked_top_view_npArr와 같은 크기면서 0으로 채워진 이미지를 변수 circle_img에 저장. 
        point_list = [] # 검사된 좌표들을 담을 빈 리스트 변수 point_list.
        
        # indexing_masked[0])에 해당 좌표들의 x, 해당 좌표들의 y가 담기게 된다.
        for index in range(len(indexing_masked[0])):           # 검출된 좌표들의 개수만큼 반복문을 돌린다.
            tmp_point = [indexing_masked[0][index], indexing_masked[1][index]]        # 일단 첫번째 좌표를 tmp_point에 저장 한다. (반복문을 돌릴때마다 임의의 좌표를 tmp_point에 담을 것임.)
            if len(point_list) == 0:            # 만약 point_list에 담긴 좌표가 없다면,
                point_list.append(tmp_point)    # 첫번째 좌표 tmp_point를 point_list에 추가해준다.
            else:                               # 반대로 point_list에 좌표가 담겨 있다면,
                min_distance = 999              # 일단 최소 거리를 999로 설정한다.
                for point in point_list:                                           # point_list안에 들은 point들에 대하여:
                     distance = self.get_distance_from_two_points(point, tmp_point)   # 임시 좌표 tmp_point와 반복문에서 돌아가는 point와의 거리를 distance에 저장한다.
                     if min_distance>distance:                                        # 만약 최소 거리 min_distance가 위에서 잰 거리보다 크다면:
                         min_distance = distance                                        # min_distance를 distance로 갱신.
                if min_distance > standard_pixel_distance:                         # 만약 min_distance가 standard_pixel_distance보다 크다면:
                    point_list.append(tmp_point)                                        # tmp_point를 point_list에 추가한다.         
                # 따라서 위 반복문 코드의 내용은 point_list에 들은 좌표들과 tmp_point의 최소 거리를 구하여 최소 거리 min_distance가 우리가 설정한 거리 standard_pixel_distance 보다 클 때 point_list에 추가해주는 원리이다.     
                # 결국 일정 거리 이상 떨어진 좌표만 point_list에 추가해주는 코드이다.   
            # 만약 검출된 좌표점의 개수가 standard_pixel_recommend_max_num보다 많으면 break로 멈춤. 에러방지.
            if standard_pixel_recommend_max_num < len(point_list):
                break

        for index, point in enumerate(point_list):
            # 리사이즈된 좌표들을 원본 이미지 크기에 맞게 다시 복원.
            point[0] *= optim_resize_const
            point[1] *= optim_resize_const
            # 좌표 리스트에 복원된 좌표들을 추가.
            point_list[index] = point 
        #아래는 주석이므로 사용안함.
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
        return point_list   # 좌표 리스트들을 반환.
    # field_image_mask는 gui의 1번째 화면에서 아랫줄 1->2->3 화면으로 가는 과정
    def field_image_mask(self, top_view_npArr, field_minimum_condition, field_maximum_condition, roi_size, field_contour_dilate_p, field_contour_erode_p, gui_param_image):
    # field_image_mask의 매개변수는 gui의 첫번째 화면인 top_view_npArr과, 필드 최소, 최대범위인 field_minimum_condition, field_maximum_condition 그리고
    # 필드 팽창 및 침식강도인 field_contour_dilate_p, field_contour_erode_p 마지막으로 gui와 통신할때 전송하는 매개변수 리스트인 gui_param_image.
    
        top_view_npArr = cv2.cvtColor(top_view_npArr, cv2.COLOR_BGR2HSV) # HSV 변환
        contour_img = cv2.inRange(top_view_npArr, field_minimum_condition, field_maximum_condition) # inRange를 이용하여 top_view_npArr에서 필드 최소~최대 범위 사이의 값만 contour_img에 이진 이미지로써 저장한다. 
        gui_param_image["bin_field_img"] = contour_img # gui_param_image의 bin_field_img에 contour_img 저장. ( 이 때 bin_field_img는 gui 하단 첫번째 이미지 )

        contours, _ = cv2.findContours(contour_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # gui 화면 내에서 폐곡선을 찾아주는 모듈 findContours
        contour_img[:,:] = np.array(0)  #  contour_img를 0으로 채워 초기화 ( 윗줄에서 contours에 저장해서 이제 딱히 쓸모 없어져서 다시 초기화 해준 것.)
        for contour in contours:
            cv2.drawContours(contour_img, [contour],0, (255, 255, 255),3) # drawContours 모듈을 이용하여 경기장 화면에 폐곡선의 경계선을 하얀색으로 그려준다.
        cv2.fillPoly(contour_img, pts=contours, color=(255,255,255))      # fillPoly 모듈을 이용하여 폐곡선 빈 곳들을 하얀색으로 칠해준다. 
        # 결국 검은색 도화지에 폐곡선들의 경계선을 하얀색으로 그려주고 그 내부를 하얀색으로 채우면 남은 검은색 부분이 자연스럽게 경기장의 line이 되는 것. 
        contour_img = contour_img[:, roi_size["pre_x"]:roi_size["pre_x"]+roi_size["x_size"]] # 컨투어 이미지에 ROI 처리를 해준다.
        gui_param_image["resize_field_img"] = contour_img # gui_param_image의 resize_field_img에 컨투어 이미지를 저장한다.
        
        # 마지막으로 gui 하단의 마지막 화면을 만들기 위해 팽창과 침식을 이용한다.
        # 먼저 하얀색 면적의 팽창강도를 높게 설정하여 line을 모두 지우고, 넓어진 하얀색 면적을 다시 원래대로 검은색으로 채우기 위해 침식 강도도 그에 못지 않은 크기로 설정하는 원리.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (field_contour_dilate_p, field_contour_dilate_p))
        contour_img = cv2.dilate(contour_img, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (field_contour_erode_p, field_contour_erode_p))
        contour_img = cv2.erode(contour_img, kernel)
        gui_param_image["erode_dilate_field"] = contour_img  # gui_param_image의 erode_dilate_field에 contour_img 저장.

        return contour_img, gui_param_image
# gui 실행 함수.
def run_gui(q, paramq, gui_param_message_form, gui_param_image_form):
#  gui_param_message_form는 gui에서 이 코드로 보내는 자료들. gui_param_image_form는 이 코드에서 gui로 보내는 자료들. paramq는 자료구조에서 큐를 의미.
#  밑의 자료는 gui를 화면에 띄우기 위해 Pyqt를 이용한 내용임.
    app = QApplication(sys.argv)   # gui 창을 띄우기 위한 모듈 QApplication
    mywindow = bh_GUI.App(q, paramq, gui_param_message_form, gui_param_image_form)
    mywindow.show()
    app.exec_()

# YOLO에서 이미지를 받았을 때, when_receive_yolo_image 함수가 작동. 
def when_receive_yolo_image(ros_data, args):
    #start = time.time()
    # args라는 list 파라미터들을 통해 초기화
    priROS = args[0]
    useful_function = args[1]
    q = args[2]
    paramq = args[3]
    
    # 변수를 주고 받기 위해 전역변수로 설정
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
    
    # 몬테카를로 알고리즘에 보낼 변수들.
    mcl_message = {"debug_num":1,
                    "sensor_data_x":[],      # mcl에서 인식된 좌표들의 x값들
                    "sensor_data_y":[],      # mcl에서 인식된 좌표들의 y값들
                    "op3_local_mode": False,  # op3한테 받은 내용을 mcl에 전송하는 변수
                    "xy_distribution":0,          # 추정된 좌표에서 얼마만큼 분산할 것인가?
                    "orien_distribution":0,        # 추정된 좌표에서 얼마만큼 회전하여 분산할 것인가?
                    "diff_limit_wslow_wfast":0,     # line과 검출된 좌표간의 괴리감에 비례하여 커지는 변수. xy_distribution과 곱해지므로써 점점 더 분산됨.
                    "start_point_x":0,           # 분산 전 시작 좌표 x
                    "start_point_y":0,           # 분산 전 시작 좌표 y
                    "start_point_orien":0}       # 분산 전 시작 각도 orien
    
    # gui_param_image는 gui에 보낼 이미지의 변수들을 받아서 초기화 해준다.
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

    # 여기부턴 이미지 변환 ( 압축된 이미지를 Uncompressed 해줌)
    np_arr = np.fromstring(ros_data.data, np.uint8) 
    view_npArr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
    top_view_npArr = useful_function.perspective(view_npArr, pre_processing_size)  # 로봇의 시야로 받아온 view_npArr을 perspective로 변환하여 top_view_npArr에 저장.
    gui_param_image["yolo_processed_img"] = top_view_npArr   # gui_param_image의 yolo_processed_img에 top_view_npArr저장.
    
    # 팽창과 침식을 통해 image_mask 함수들에서 폐곡선을 받아와서 각각 contour_img에 하단의 필드 영역 gui 받아서 저장  // masked_top_view_npArr에 상단의 경기장+라인 이진 이미지 받아서 저장. 
    contour_img, gui_param_image = useful_function.field_image_mask(top_view_npArr, field_minimum_condition, field_maximum_condition, roi_size, field_contour_dilate_p, field_contour_erode_p, gui_param_image)
    masked_top_view_npArr, gui_param_image = useful_function.Image_mask(top_view_npArr, contour_img, mask_minimum_condition, mask_maximum_condition, roi_size, dilate_power, erode_power, gui_param_image)
    
    # masked_top_view_npArr의 요소들(0,1)을 optim_resize_const로 나눠준다. 예로 450을 3으로 나눠주면 150으로 리사이즈. (연산 부담을 줄여줌)
    optim_resize_masked_top_view_npArr = cv2.resize(masked_top_view_npArr, 
                                                    dsize = (round(np.shape(masked_top_view_npArr)[1]/optim_resize_const), round(np.shape(masked_top_view_npArr)[0]/optim_resize_const)),
                                                    interpolation=cv2.INTER_AREA)    
    # dsize는 사이즈 정의 / round는 float을 int로 바꿔주는 것. / INTER_AREA는 리사이즈 방법인데, 이미지 사이즈 조정할때 최적화된 방식이다.
    indexing_masked = np.where(optim_resize_masked_top_view_npArr>254) # optim_resize_masked_top_view_npArr에서 하얀색인 부분들의 좌표들을 찾아 indexing_masked에 저장한다.

    point_list = []
    # standard_pixel_distance는 라인에서 좌표점을 추출할 때, 점과 점사이의 거리를 조정하는 것. 즉 standard_pixel_distance와 추출되는 좌표점의 개수는 반비례한다.
    # standard_pixel_max_distance와standard_pixel_min_distance는 gui에서 가변 거리 필터 설정의 최대, 최소 거리를 의미하며, 두 변수를 조정하여 추출되는 좌표점의 개수를 22~23개로 제한하는 원리.
    # standard_pixel_distance_unit는 검사 감소 거리로써 좌표점의 개수를 한번에 얼마만큼 조정할 것인지에 대한 변수이다. ( ex: 27->25->23 )
    if standard_pixel_max_distance >= standard_pixel_min_distance:
        if standard_pixel_distance > standard_pixel_max_distance:
            standard_pixel_distance = standard_pixel_max_distance # standard_pixel_distance가 너무 커지면 그 크기를 줄이고, 
        elif standard_pixel_distance < standard_pixel_min_distance:
            standard_pixel_distance = standard_pixel_min_distance # standard_pixel_distance가 너무 작아지면 그 크기를 늘리는 원리.
            
         # 좌표들을 받아와서 point_list에 저장.
        point_list = useful_function.get_profit_point(optim_resize_masked_top_view_npArr, indexing_masked, standard_pixel_distance, standard_pixel_recommend_max_num, optim_resize_const)
        if len(point_list)  < standard_pixel_recommend_min_num:      # recommend_min_num 보단 커야하며
            standard_pixel_distance -= standard_pixel_distance_unit    # 좌표간의 거리 조절
        elif len(point_list) > standard_pixel_recommend_max_num:     # recommend_max_num 보단 작아야한다.
            standard_pixel_distance += standard_pixel_distance_unit    # 좌표간의 거리 조절
   
    gui_param_image["num_of_line_point"] = len(point_list) # num_of_line_point에 검출된 좌표의 개수를 저장.
    
    circle_img = np.zeros_like(masked_top_view_npArr, np.uint8)  # 검출된 좌표들에 동그라미를 그려주기 위한 검은색 빈 이미지
    # 검출된 좌표들의 위치에 openCV를 이용해 좌표들에 동그라미를 그려준다
    for point in point_list:
        cv2.circle(circle_img, (point[1], point[0]), 5, 255)
    gui_param_image["distance_line"] = circle_img  # distance_line 변수에 검출된 모든 좌표들을 표시한 circle_img 저장.
    # 너무 많은 좌표들이 검출되지 않기 위한 안전장치.
    if len(point_list) < standard_pixel_limit_num:
        for point in point_list:
            mcl_message["sensor_data_y"].append(point[0]-(pre_processing_size//2))  # 검출된 y 좌표들을 보낼 메세지 변수 sensor_data_y
            mcl_message["sensor_data_x"].append(point[1]+roi_push_move)             # 검출된 x 좌표들을 보낼 메세지 변수 sensor_data_x
        for i in range(standard_pixel_limit_num-len(point_list)):
            mcl_message["sensor_data_x"].append(-100)             # 여유 공간이 남으면 좌표값은 -100으로 채워짐.
            mcl_message["sensor_data_y"].append(-100)             # 여유 공간이 남으면 좌표값은 -100으로 채워짐.  mcl에 가면 -100은 처리가 안됨.
        # op3에게 받은 정보를 그대로 저장하는 것. 시작점, 분산 정도, 각도 , 괴리감 , 로컬모드.    
        mcl_message["op3_local_mode"] = op3_local_mode          
        mcl_message["start_point_x"] = start_point_x
        mcl_message["start_point_y"] = start_point_y
        mcl_message["start_point_orien"] = start_point_orien
        mcl_message["xy_distribution"] = start_point_xy_distribution
        mcl_message["orien_distribution"] = start_point_orien_distribution
        mcl_message["diff_limit_wslow_wfast"] = start_point_diff_limit_wslow_wfast
        # 검출 좌표 개수는 전송해야되는 최소 개수 standard_pixel_send_min_num 보다 커야되고 standard_pixel_send_max_num 보다 작아야함( 큰 의미없음.) 
        # 마지막으로 op3_local_mode가 True 라는 것은, 로봇이 걸음을 멈추고 고개를 내려 localization할 준비를 마쳤다는 것을 의미.
        if len(point_list) > standard_pixel_send_min_num and len(point_list) < standard_pixel_send_max_num and op3_local_mode is True:
            if standard_pixel_send_wait_count < standard_pixel_im_ready_count:        # op3가 멈추고 고개를 내릴만큼의 시간을 기다리는 것.
                mcl_message["op3_local_mode"] = op3_local_mode # mcl에 보낼 변수이므로 mcl 메세지 속에 잘 저장해준다.
                priROS.talker(mcl_message)                     # priROS와 talker를 이용해 mcl_message를 전송.
            else:
                standard_pixel_im_ready_count += 1  # 시간내에 위의 조건들을 모두 만족시켜, count가 부족하면 count를 증가시킴.
        elif op3_local_mode is False:               # op3_local_mode가 False가 되어 로봇이 움직이기 시작했을 떈
            standard_pixel_im_ready_count = 0      #  기다리면 안되므로 count값을 0으로 만들어줌.

    priROS.talker_head(robot_desire_tilt, len(point_list))  # 로봇한테 이 tilt 값으로 고개를 조정해라는 명령과, 그리고 감지된 좌표 개수를 말해줌.
    #허공을 보고있을 경우 op3에게 알려 op3가 localization 결과를 기다리지 않고 다시 움직이게 만들기 위함.
    
    # 큐에 그냥 put을 하면 데드락이 걸리기 때문에, put_nowait를 써서 gui가 별개로 돌아가게 만든다
    gui_param_image["op3_local_mode"] = op3_local_mode
   
    try:
        q.put_nowait(gui_param_image)  # q가 차있을 경우 put을 넣으면 q가 될때까지 기다려서 렉이 걸리기 때문에 기다리지 않기 위한 방지용 코드.
    except Exception as e:             # q가 차있으면 안넣고, q가 비어있으면 넣는 방식
        pass
    
    #gui에서 사람이 입력한 정보들을 받아오기 위한 코드
    try:
        param_dic = paramq.get_nowait()  # paramq가  비어있더라도 찰때까지 기다리지 않기 위한 코드.
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
        # gui에서 얻어온 정보를 numpy 형태로 바꿔줘야 하므로 하단 4줄에서 형태를 바꿔줌.
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
    # op3로 부터 얻은 정보들을 다시 넣어주는 과정.
    global op3_local_mode
    global start_point_x
    global start_point_y
    global start_point_orien
    op3_local_mode = ros_data.op3_local_mode
    start_point_x = ros_data.start_point_x
    start_point_y = ros_data.start_point_y
    start_point_orien = ros_data.start_point_orien


if __name__=='__main__':
    priROS = priROS() # ROS에서 쓰는 함수들을 모아놓은 priROS 클래스 초기화
    useful_function = useful_function() #유용한 함수들을 class로 묶은 것이 useful_function. 메인문에서 초기화.
    q = Queue(1)                   # queue 자료구조 변수 q
    paramq = Queue(1)              # queue 자료구조 변수 paramq
    # p는 서브process로써 파이썬에서 별도로 생성한 것. args에 4개의 매개변수는 q, paramq 그리고 gui와 주고받을 메세지이다. daemon=True는 main process가 죽으면 같이 죽는 프로세스.
    # Process의 이름은 producer 이며, 프로세스에서 실행되는 함수는 run_gui이다.
    p = Process(name="producer", target=run_gui, args=(q, paramq, gui_param_message_form, gui_param_image_form), daemon=True)
    p.start() # 프로세스 시작.
    rospy.init_node('kudos_vision_local_process', anonymous = False)                          #노드 초기화
    rospy.Subscriber("/output/image_raw/compressed", CompressedImage, when_receive_yolo_image,(priROS, useful_function, q, paramq), queue_size=1) # 이미지를 받는 subscriber
    rospy.Subscriber("kudos_vision_op3_local_mode", kvolm, when_receive_op3_local_msg,(priROS, ), queue_size=1)        # op3 메세지를 받는 subscriber 
    # 각각 메세지를 받을 때, 각각 when_receive_yolo_image,(priROS, useful_function, q, paramq)  when_receive_op3_local_msg,(priROS, )를 실행. 인자는 괄호 안의 인자들이 들어감.
    rospy.spin()
