from logging import exception
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtGui import QIcon
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PIL.ImageQt import ImageQt 
import datetime
try:
    import sys
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except Exception as e:
    pass
import cv2
from multiprocessing import Process, Queue
import multiprocessing as mp
import numpy as np
import os 
os.environ['MPLCONFIGDIR'] = os.getcwd() + "/configs/"
import matplotlib.pyplot as plt

class App(QWidget):
    # gui를 호출하면 제일먼저 init이 호출되어 self 인자들을 선언하여 만들어줌.
    def __init__(self, dataQueue, parameterQueue, gui_param_message_form, gui_param_image_form):
        super().__init__()
        self.img_size = 195
        self.hsv_img = 0
        self.IS_GUI_SYNC = True
        self.dataQueue = dataQueue
        self.parameterQueue = parameterQueue
        self.gui_param_message = gui_param_message_form
        self.gui_param_imgage = gui_param_image_form
        self.initUI()
    
    # gui 띄우는 함수
    def initUI(self):   # 인자들을 self로 받은 이유는, 
        self.setWindowTitle('라인 검출 GUI') # gui 제목
        self.center()                       # gui가 중앙에 뜸.
        self.resize(1000, 1000)             # gui 창의 크기를 1000,1000으로 제한.
        # QLabel을 통해 내부에 라벨을 붙일 수 있음.
        label_min_HSV = QLabel('HSV 최소 범위', self)   
        label_min_HSV.move(20, 20)
        miniLabel = QLabel('H', self)
        miniLabel.move(10, 45)
        # QLineEdit은 대입 가능한 박스.
        self.min_H = QLineEdit(self)
        self.min_H.move(25, 45)
        miniLabel = QLabel('S', self)
        miniLabel.move(10, 70)
        self.min_S = QLineEdit(self)
        self.min_S.move(25, 70)
        miniLabel = QLabel('V', self)
        miniLabel.move(10, 95)
        self.min_V = QLineEdit(self)
        self.min_V.move(25, 95)

        label_max_HSV = QLabel('HSV 최대 범위', self)
        label_max_HSV.move(20, 120)
        miniLabel = QLabel('H', self)
        miniLabel.move(10, 145)
        self.max_H = QLineEdit(self)
        self.max_H.move(25, 145)
        miniLabel = QLabel('S', self)
        miniLabel.move(10, 170)
        self.max_S = QLineEdit(self)
        self.max_S.move(25, 170)
        miniLabel = QLabel('V', self)
        miniLabel.move(10, 195)
        self.max_V = QLineEdit(self)
        self.max_V.move(25, 195)

        label_pre_processing_size = QLabel('처리 전 변환할 이미지 사이즈', self)
        label_pre_processing_size.move(20, 230)
        self.pre_process_size = QLineEdit(self)
        self.pre_process_size.move(25, 250)

        label_roi_size = QLabel('ROI 사이즈 설정', self)
        label_roi_size.move(20, 280)
        roiLabel = QLabel('x_size', self)
        roiLabel.move(10, 305)
        self.roi_x_size = QLineEdit(self)
        self.roi_x_size.move(90, 305)
        roiLabel = QLabel('y_size', self)
        roiLabel.move(10, 330)
        self.roi_y_size = QLineEdit(self)
        self.roi_y_size.move(90, 330)
        roiLabel = QLabel('pre_x', self)
        roiLabel.move(10, 355)
        self.roi_pre_x = QLineEdit(self)
        self.roi_pre_x.move(90, 355)
        roiLabel = QLabel('push_move', self)
        roiLabel.move(10, 380)
        self.roi_push_move = QLineEdit(self)
        self.roi_push_move.move(90, 380)

        label_dil_ero = QLabel('침식 강도와 팽창강도 설정', self)
        label_dil_ero.move(20, 410)
        dil_ero = QLabel('팽창강도', self)
        dil_ero.move(10, 430)
        self.dilate_power = QLineEdit(self)
        self.dilate_power.move(70, 430)
        dil_ero = QLabel('침식강도', self)
        dil_ero.move(10, 450)
        self.erode_power = QLineEdit(self)
        self.erode_power.move(70, 450)

        label_pixel_distance = QLabel('가변 거리 필터 설정(22~23개)', self)
        label_pixel_distance.move(20, 480)
        label_pixel_distance = QLabel('최대거리', self)
        label_pixel_distance.move(10, 500)
        self.standard_pixel_max_distance = QLineEdit(self)
        self.standard_pixel_max_distance.move(100, 500)
        label_pixel_distance = QLabel('최소거리', self)
        label_pixel_distance.move(10, 520)
        self.standard_pixel_min_distance = QLineEdit(self)
        self.standard_pixel_min_distance.move(100, 520)
        label_pixel_distance = QLabel('검사감소거리', self)
        label_pixel_distance.move(10, 540)
        self.standard_pixel_distance_unit = QLineEdit(self)
        self.standard_pixel_distance_unit.move(100, 540)

        self.label_gaussian_filter = QLabel('가우시안 필터 사이즈 설정', self)
        self.label_gaussian_filter.move(400, 20)
        self.gaussian_filter = QLineEdit(self)
        self.gaussian_filter.move(400,40)

        self.label_perspective = QLabel('투영 변환 설정', self)
        self.label_perspective.move(400, 70)
        self.label_perspective = QLabel('옆으로 늘림 배수', self)
        self.label_perspective.move(400, 90)
        self.empty_size_mul = QLineEdit(self)
        self.empty_size_mul.move(540, 90)
        self.label_perspective = QLabel('상단 왼쪽 위치 배수', self)
        self.label_perspective.move(400,110)
        self.cp_size_left_loc = QLineEdit(self)
        self.cp_size_left_loc.move(540, 110)
        self.label_perspective = QLabel('상단 오른쪽 위치 배수', self)
        self.label_perspective.move(400, 130)
        self.cp_size_right_loc = QLineEdit(self)
        self.cp_size_right_loc.move(540, 130)

        self.label_desire_tilt = QLabel('고개 숙인 각도(degree)', self)
        self.label_desire_tilt.move(400, 160)
        self.tilt_degree = QLineEdit(self)
        self.tilt_degree.move(400, 180)

        self.label_f_min_hsv = QLabel("필드 최소 hsv 범위", self)
        self.label_f_min_hsv.move(400, 210)
        self.label_f_min_h = QLabel("H", self)
        self.label_f_min_h.move(400, 230)
        self.f_min_H = QLineEdit(self)
        self.f_min_H.move(420, 230)
        self.label_f_min_S = QLabel("S", self)
        self.label_f_min_S.move(400,250)
        self.f_min_S = QLineEdit(self)
        self.f_min_S.move(420, 250)
        self.label_f_min_v = QLabel("V",self)
        self.label_f_min_v.move(400,270)
        self.f_min_V = QLineEdit(self)
        self.f_min_V.move(420, 270)

        self.label_f_max_hsv = QLabel("필드 최대 hsv 범위", self)
        self.label_f_max_hsv.move(400,300)
        self.label_f_max_h = QLabel("H", self)
        self.label_f_max_h.move(400, 320)
        self.f_max_H = QLineEdit(self)
        self.f_max_H.move(420, 320)
        self.label_f_max_s = QLabel("S", self)
        self.label_f_max_s.move(400,340)
        self.f_max_S = QLineEdit(self)
        self.f_max_S.move(420, 340)
        self.label_f_max_v = QLabel("V",self)
        self.label_f_max_v.move(400,360)
        self.f_max_V = QLineEdit(self)
        self.f_max_V.move(420, 360)

        label_f_dil_ero = QLabel('필드 침식 강도와 팽창강도 설정', self)
        label_f_dil_ero.move(400, 390)
        dil_ero = QLabel('팽창강도', self)
        dil_ero.move(400, 410)
        self.f_dilate_power = QLineEdit(self)
        self.f_dilate_power.move(470, 410)
        dil_ero = QLabel('침식강도', self)
        dil_ero.move(400, 430)
        self.f_erode_power = QLineEdit(self)
        self.f_erode_power.move(470, 430)

        label_start_point_normal_distribution = QLabel("로컬 시작 정규 분포 설정", self)
        label_start_point_normal_distribution.move(400, 460)
        label_start_point_normal_distribution = QLabel("파티클 거리 분산", self)
        label_start_point_normal_distribution.move(400, 480)
        self.start_point_xy_distribution = QLineEdit(self)
        self.start_point_xy_distribution.move(520, 480)
        label_start_point_normal_distribution = QLabel("파티클 회전 분산", self)
        label_start_point_normal_distribution.move(400, 500)
        self.start_point_orien_distribution = QLineEdit(self)
        self.start_point_orien_distribution.move(520, 500)
        label_start_point_normal_distribution = QLabel("모드 변경 제한 변수(wslow/wfast)", self)
        label_start_point_normal_distribution.move(400, 530)
        self.start_point_diff_limit_wslow_wfast = QLineEdit(self)
        self.start_point_diff_limit_wslow_wfast.move(620, 530)

        label_setting_point_list_num = QLabel("감지 포인트 개수 설정", self)
        label_setting_point_list_num.move(720, 160)
        label_setting_point_list_num = QLabel("전송최소개수",self)
        label_setting_point_list_num.move(720, 180)
        self.standard_pixel_send_min_num = QLineEdit(self)
        self.standard_pixel_send_min_num.move(820, 180)
        label_setting_point_list_num = QLabel("전송최대개수",self)
        label_setting_point_list_num.move(720, 200)
        self.standard_pixel_send_max_num = QLineEdit(self)
        self.standard_pixel_send_max_num.move(820, 200)
        label_setting_point_list_num = QLabel("대기카운트",self)
        label_setting_point_list_num.move(720, 220)
        self.standard_pixel_send_wait_count = QLineEdit(self)
        self.standard_pixel_send_wait_count.move(820, 220)
        

        self.label_op3_local_mode = QLabel("op3_local_mode: False", self)
        self.label_op3_local_mode.move(600, 20)
        self.label_num_of_line_point = QLabel("감지된 라인 좌표:", self)
        self.label_num_of_line_point.move(600, 40)
        self.num_of_line_point = QLabel("00",self)
        self.num_of_line_point.move(710, 40)
        # 이미지를 넣을 빈 박스들 생성.
        self.img_0_label = QLabel(self)
        self.img_0_label.setGeometry(0, 600, self.img_size, self.img_size)
        self.img_1_label = QLabel(self)
        self.img_1_label.setGeometry(200, 600, self.img_size, self.img_size)
        self.img_2_label = QLabel(self)
        self.img_2_label.setGeometry(400, 600, self.img_size, self.img_size)
        self.img_3_label = QLabel(self)
        self.img_3_label.setGeometry(600, 600, self.img_size, self.img_size)
        self.img_4_label = QLabel(self)
        self.img_4_label.setGeometry(800, 600, self.img_size, self.img_size)
        self.img_5_label = QLabel(self)
        self.img_5_label.setGeometry(0, 800, self.img_size, self.img_size)
        self.img_6_label = QLabel(self)
        self.img_6_label.setGeometry(200, 800, self.img_size, self.img_size)
        self.img_7_label = QLabel(self)
        self.img_7_label.setGeometry(400, 800, self.img_size, self.img_size)
        

        # 함수랑 연결된 버튼을 만들어줌. 버튼 누를시 해당 함수 실행.
        self.btn_load_data_and_set = QPushButton('데이터 로드 및 적용', self)  
        self.btn_load_data_and_set.move(200, 50)
        self.btn_load_data_and_set.clicked.connect(self.load_data_func)
        self.btn_save_data_and_set = QPushButton('데이터 세이브 및 적용', self)
        self.btn_save_data_and_set.move(200, 100)
        self.btn_save_data_and_set.clicked.connect(self.save_data_func)
        self.btn_sync_video = QPushButton('gui 동기화', self)
        self.btn_sync_video.move(200, 150)
        self.btn_sync_video.clicked.connect(self.sync_video_func)
        self.btn_sync_video = QPushButton('gui 비동기화', self)
        self.btn_sync_video.move(200, 200)
        self.btn_sync_video.clicked.connect(self.un_sync_video_func)
        self.btn_debuging_img = QPushButton('이미지 디버깅', self)
        self.btn_debuging_img.move(200,250)
        self.btn_debuging_img.clicked.connect(self.debug_img)
       
        # 상단의 코드들이 반복되게 해놓음.
        self.timer = QTimer(self)
        self.timer.start(0)
        self.timer.timeout.connect(lambda: self.timeout_run())
     
    #위의 타이머 코드가 끝나게 되면 실행하는 함수.
    def timeout_run(self):
        if self.IS_GUI_SYNC:
            gui_param_image = self.dataQueue.get()   # self.dataQueue에 있는 data들을 가져온다. 그 data들은 q의 data와 같음.
            try:
                pixmap_img0 =  self.convert_nparray_to_Qpixmap(gui_param_image["yolo_processed_img"])
                self.img_0_label.setPixmap(pixmap_img0)
                self.hsv_img = gui_param_image["hsv_img"]
                pixmap_img1 =  self.convert_nparray_to_Qpixmap(gui_param_image["bin_line_img"])
                self.img_1_label.setPixmap(pixmap_img1)
                pixmap_img2 =  self.convert_nparray_to_Qpixmap(gui_param_image["resize_img"])
                self.img_2_label.setPixmap(pixmap_img2)
                pixmap_img3 =  self.convert_nparray_to_Qpixmap(gui_param_image["erode_dilate_line"])
                self.img_3_label.setPixmap(pixmap_img3)
                pixmap_img4 =  self.convert_nparray_to_Qpixmap(gui_param_image["distance_line"])
                self.img_4_label.setPixmap(pixmap_img4)

                pixmap_img5 =  self.convert_nparray_to_Qpixmap(gui_param_image["bin_field_img"])
                self.img_5_label.setPixmap(pixmap_img5)
                pixmap_img6 =  self.convert_nparray_to_Qpixmap(gui_param_image["resize_field_img"])
                self.img_6_label.setPixmap(pixmap_img6)
                pixmap_img7 =  self.convert_nparray_to_Qpixmap(gui_param_image["erode_dilate_field"])
                self.img_7_label.setPixmap(pixmap_img7)

                if gui_param_image["op3_local_mode"] == False:
                    self.label_op3_local_mode.setText("op3_local_mode: False")
                elif gui_param_image["op3_local_mode"] == True:
                    self.label_op3_local_mode.setText("op3_local_mode: True")
                detect_point_str = str(gui_param_image["num_of_line_point"])
                self.num_of_line_point.setText(detect_point_str)     # setText를 이용해 검출된 좌표 개수를 gui에 실시간 업데이트

            except Exception as e:
                print("Error!", e, "by_bh")
            
            # 사람이 gui에 적어놓은 변수들을 int형으로 변환하여 local_process_gui2에 보낸다.
            try:
                # minipack은 리스트의 내용들을 변수 하나에 묶어준 것.
                minipack = [int(self.min_H.text()),int(self.min_S.text()),int(self.min_V.text())]
                self.gui_param_message["mask_minimum_condition"] = minipack
                minipack = [int(self.max_H.text()),int(self.max_S.text()),int(self.max_V.text())]
                self.gui_param_message["mask_maximum_condition"] = minipack
                self.gui_param_message["pre_processing_size"] = int(self.pre_process_size.text())
                self.gui_param_message["roi_size"]["x_size"] = int(self.roi_x_size.text())
                self.gui_param_message["roi_size"]["y_size"] = int(self.roi_y_size.text())
                self.gui_param_message["roi_size"]["pre_x"] = int(self.roi_pre_x.text())
                self.gui_param_message["roi_push_move"] = int(self.roi_push_move.text())
                self.gui_param_message["dilate_power"] = int(self.dilate_power.text())
                self.gui_param_message["erode_power"] = int(self.erode_power.text())
                self.gui_param_message["guassian_filter_size"] = int(self.gaussian_filter.text())
                self.gui_param_message["empty_size_mul"] = float(self.empty_size_mul.text())
                self.gui_param_message["cp_size_left_loc"] = float(self.cp_size_left_loc.text())
                self.gui_param_message["cp_size_right_loc"] = float(self.cp_size_right_loc.text())
                self.gui_param_message["tilt_degree"] = int(self.tilt_degree.text())
                minipack = [int(self.f_min_H.text()),int(self.f_min_S.text()),int(self.f_min_V.text())]
                self.gui_param_message["field_minimum_condition"] = minipack
                minipack = [int(self.f_max_H.text()),int(self.f_max_S.text()),int(self.f_max_V.text())]
                self.gui_param_message["field_maximum_condition"] = minipack
                self.gui_param_message["field_contour_dilate_p"] = int(self.f_dilate_power.text())
                self.gui_param_message["field_contour_erode_p"] = int(self.f_erode_power.text())
                self.gui_param_message["standard_pixel_max_distance"] = int(self.standard_pixel_max_distance.text())
                self.gui_param_message["standard_pixel_min_distance"] = int(self.standard_pixel_min_distance.text())
                self.gui_param_message["standard_pixel_distance_unit"] = int(self.standard_pixel_distance_unit.text())
                self.gui_param_message["standard_pixel_send_min_num"] = int(self.standard_pixel_send_min_num.text())
                self.gui_param_message["standard_pixel_send_max_num"] = int(self.standard_pixel_send_max_num.text())
                self.gui_param_message["standard_pixel_send_wait_count"] = int(self.standard_pixel_send_wait_count.text())
                self.gui_param_message["start_point_xy_distribution"] = float(self.start_point_xy_distribution.text())
                self.gui_param_message["start_point_orien_distribution"] = float(self.start_point_orien_distribution.text())
                self.gui_param_message["start_point_diff_limit_wslow_wfast"] = float(self.start_point_diff_limit_wslow_wfast.text())

               # 최종적으로 완성된 gui_param_message를 parameterQueue에 넣어준다.
                self.parameterQueue.put_nowait(self.gui_param_message)
            except Exception as e:
                print(e)
     
    # 이미지 디버깅 버튼 ( 누르면 이미지 나옴.)
    def debug_img(self):
        plt.imshow(self.hsv_img)
        plt.show()
    
    # 데이터 로드 및 적용 ( gui에 있는 버튼을 누르면 saved_configuration.txt에 들은 내용을 gui에 적용시킴. )
    def load_data_func(self):
        f = open("./bh_function/bh_data/saved_configuration.txt", 'r')
        data_list = f.readlines()
        f.close()
        for index, data in enumerate(data_list):
            try:
                data = data.replace("\n","")
                data_list[index] = data
            except Exception as e:
                pass

        try:
            self.min_H.setText(data_list[0])
            self.min_S.setText(data_list[1])
            self.min_V.setText(data_list[2])
            self.max_H.setText(data_list[3])
            self.max_S.setText(data_list[4])
            self.max_V.setText(data_list[5])
            self.pre_process_size.setText(data_list[6])
            self.roi_x_size.setText(data_list[7])
            self.roi_y_size.setText(data_list[8])
            self.roi_pre_x.setText(data_list[9])
            self.roi_push_move.setText(data_list[10])
            self.dilate_power.setText(data_list[11])
            self.erode_power.setText(data_list[12])
            self.gaussian_filter.setText(data_list[13])
            self.empty_size_mul.setText(data_list[14])
            self.cp_size_left_loc.setText(data_list[15])
            self.cp_size_right_loc.setText(data_list[16])
            self.tilt_degree.setText(data_list[17])
            self.f_min_H.setText(data_list[18])
            self.f_min_S.setText(data_list[19])
            self.f_min_V.setText(data_list[20])
            self.f_max_H.setText(data_list[21])
            self.f_max_S.setText(data_list[22])
            self.f_max_V.setText(data_list[23])
            self.f_dilate_power.setText(data_list[24])
            self.f_erode_power.setText(data_list[25])
            self.standard_pixel_max_distance.setText(data_list[26])
            self.standard_pixel_min_distance.setText(data_list[27])
            self.standard_pixel_distance_unit.setText(data_list[28])
            self.start_point_xy_distribution.setText(data_list[29])
            self.start_point_orien_distribution.setText(data_list[30])
            self.start_point_diff_limit_wslow_wfast.setText(data_list[31])
            self.standard_pixel_send_min_num.setText(data_list[32])
            self.standard_pixel_send_max_num.setText(data_list[33])
            self.standard_pixel_send_wait_count.setText(data_list[34])
        except Exception as e:
            pass
        QMessageBox.about(self, "message", "데이터 로드와 적용!")
    
    # gui에 적힌 data들을 저장하는 '데이터 저장' 버튼
    def save_data_func(self):
        f = open('./bh_function/bh_data/saved_configuration.txt', 'w')  # ./bh_function/bh_data/saved_configuration.txt 파일을 연다.
        # minipack에 gui에 적힌 데이터들을 추가한다.
        minipack = []
        minipack.append(self.min_H.text())
        minipack.append(self.min_S.text())
        minipack.append(self.min_V.text())
        minipack.append(self.max_H.text())
        minipack.append(self.max_S.text())
        minipack.append(self.max_V.text())
        minipack.append(self.pre_process_size.text())
        minipack.append(self.roi_x_size.text())
        minipack.append(self.roi_y_size.text())
        minipack.append(self.roi_pre_x.text())
        minipack.append(self.roi_push_move.text())
        minipack.append(self.dilate_power.text())
        minipack.append(self.erode_power.text())
        minipack.append(self.gaussian_filter.text())
        minipack.append(self.empty_size_mul.text())
        minipack.append(self.cp_size_left_loc.text())
        minipack.append(self.cp_size_right_loc.text())
        minipack.append(self.tilt_degree.text())
        minipack.append(self.f_min_H.text())
        minipack.append(self.f_min_S.text())
        minipack.append(self.f_min_V.text())
        minipack.append(self.f_max_H.text())
        minipack.append(self.f_max_S.text())
        minipack.append(self.f_max_V.text())
        minipack.append(self.f_dilate_power.text())
        minipack.append(self.f_erode_power.text())
        minipack.append(self.standard_pixel_max_distance.text())
        minipack.append(self.standard_pixel_min_distance.text())
        minipack.append(self.standard_pixel_distance_unit.text())
        minipack.append(self.start_point_xy_distribution.text())
        minipack.append(self.start_point_orien_distribution.text())
        minipack.append(self.start_point_diff_limit_wslow_wfast.text())
        minipack.append(self.standard_pixel_send_min_num.text())
        minipack.append(self.standard_pixel_send_max_num.text())
        minipack.append(self.standard_pixel_send_wait_count.text())
        for index, data in enumerate(minipack):
            # gui 박스 안에서 엔터키를 눌러서 에러가 나는것을 방지하는 코드.
            try:
                data = data.replace("\n","")
                data = data+"\n"
                minipack[index] = data
            except Exception as e:
                pass
        f.writelines(minipack)     # 열었던  ./bh_function/bh_data/saved_configuration.txt 파일에 gui의 내용들이 담긴 minipack을 저장.
        f.close()                     
        QMessageBox.about(self, "message", "데이터 저장!")         
     
    # gui 동기화 버튼  ( 업데이트 재개)
    def sync_video_func(self):
        self.IS_GUI_SYNC = True
        QMessageBox.about(self, "message", "gui 싱크 활성화")
    # gui 비동기화 버튼 ( 업데이트 정지 , 너무 렉이 심하거나 느릴때 멈추는 용도 )
    def un_sync_video_func(self):
        self.IS_GUI_SYNC = False
        QMessageBox.about(self, "message", "gui 싱크 비활성화")
        
    # 이미지를 넣어주기 위해 Qpixmap으로 변환하는 함수.
    def convert_nparray_to_Qpixmap(self, img):
        try:
            if img.ndim == 2:
                img =  cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
            h, w, _ = img.shape
            img = cv2.resize(img, (int(self.img_size*(w/h)) ,self.img_size), interpolation = cv2.INTER_CUBIC)
            #img = cv2.resize(img, (self.img_size, self.img_size), interpolation = cv2.INTER_CUBIC)
            h, w, ch = img.shape

            qimg = QImage(img, w, h, 3*w, QImage.Format_RGB888) 
            qpixmap = QPixmap(qimg)
            return qpixmap

        except Exception as e:
            print(e,"_bhbh")
            return e
    
    # gui를 가운데에 띄우는 함수 ( 바꿀 일 없음 )
    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
