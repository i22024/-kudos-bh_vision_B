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

    def initUI(self):
        self.setWindowTitle('라인 검출 GUI')
        self.center() 
        self.resize(1000, 1000)
        label_min_HSV = QLabel('HSV 최소 범위', self)
        label_min_HSV.move(20, 20)
        miniLabel = QLabel('H', self)
        miniLabel.move(10, 45)
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

        label_pixel_distance = QLabel('추출 거리 설정',self)
        label_pixel_distance.move(20, 490)
        self.standard_pixel_distance = QLineEdit(self)
        self.standard_pixel_distance.move(20, 510)

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

        self.label_f_dil_ero = QLabel('필드 침식 강도와 팽창강도 설정', self)
        self.label_f_dil_ero.move(400, 390)
        dil_ero = QLabel('팽창강도', self)
        dil_ero.move(400, 410)
        self.f_dilate_power = QLineEdit(self)
        self.f_dilate_power.move(470, 410)
        dil_ero = QLabel('침식강도', self)
        dil_ero.move(400, 430)
        self.f_erode_power = QLineEdit(self)
        self.f_erode_power.move(470, 430)

        self.label_op3_local_mode = QLabel("op3_local_mode: False", self)
        self.label_op3_local_mode.move(600, 20)

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
        


        self.btn_load_data_and_set = QPushButton('데이터 로드 및 적용', self)
        self.btn_load_data_and_set.move(200, 50)
        self.btn_load_data_and_set.clicked.connect(self.load_data_func)# 클릭시 이벤트
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

        self.timer = QTimer(self)
        self.timer.start(0)
        self.timer.timeout.connect(lambda: self.timeout_run())

    def timeout_run(self):
        if self.IS_GUI_SYNC:
            gui_param_image = self.dataQueue.get()
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

            except Exception as e:
                print("Error!", e, "by_bh")

            try:
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
                self.gui_param_message["pixel_distance"] = int(self.standard_pixel_distance.text())
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
                self.parameterQueue.put_nowait(self.gui_param_message)
            except Exception as e:
                print(e)
        
    def debug_img(self):
        plt.imshow(self.hsv_img)
        plt.show()

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
            self.standard_pixel_distance.setText(data_list[13])
            self.gaussian_filter.setText(data_list[14])
            self.empty_size_mul.setText(data_list[15])
            self.cp_size_left_loc.setText(data_list[16])
            self.cp_size_right_loc.setText(data_list[17])
            self.tilt_degree.setText(data_list[18])
            self.f_min_H.setText(data_list[19])
            self.f_min_S.setText(data_list[20])
            self.f_min_V.setText(data_list[21])
            self.f_max_H.setText(data_list[22])
            self.f_max_S.setText(data_list[23])
            self.f_max_V.setText(data_list[24])
            self.f_dilate_power.setText(data_list[25])
            self.f_erode_power.setText(data_list[26])
        except Exception as e:
            pass
        QMessageBox.about(self, "message", "데이터 로드와 적용!")
    
    def save_data_func(self):
        f = open('./bh_function/bh_data/saved_configuration.txt', 'w')
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
        minipack.append(self.standard_pixel_distance.text())
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
        for index, data in enumerate(minipack):
            try:
                data = data.replace("\n","")
                data = data+"\n"
                minipack[index] = data
            except Exception as e:
                pass
        f.writelines(minipack)
        f.close()
        QMessageBox.about(self, "message", "데이터 저장!")

    def sync_video_func(self):
        self.IS_GUI_SYNC = True
        QMessageBox.about(self, "message", "gui 싱크 활성화")
    
    def un_sync_video_func(self):
        self.IS_GUI_SYNC = False
        QMessageBox.about(self, "message", "gui 싱크 비활성화")

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

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
