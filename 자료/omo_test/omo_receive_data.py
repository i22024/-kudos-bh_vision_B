import zmq
import zmqnumpy as znp
import numpy as np
import kudos_darknet
import time
import cv2


host_address = "tcp://192.168.215.147:9010"

class DataFormatTransfer():
    def __init__(self):
        pass
    
    def get_one_center_from_detections(self, detections, label):
        max_confidence = 0
        objectCenter = [-1.0,-1.0]
        for detections_index, detection in enumerate(detections):
            #print(detection)#(label, confidence, (left, top, right, bottom))
            if detection[0] == label:
                if detection[1]>max_confidence:
                    max_confidence = detection[1]
                    bbox = detection[2]
                    objectCenter = [bbox[0], bbox[1]]
        return objectCenter

    def get_mean_center_from_detections(self, detections, label):
        objectCenter = [-1.0,-1.0]
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

        return objectCenter

    def mapping_point_to_float_shape(self, npArr, objectCenter):
        if objectCenter != [-1.0, -1.0]:
            im_width_size = np.shape(npArr)[0]
            im_hight_size = np.shape(npArr)[1]
            objectCenter[0] = objectCenter[0]/im_width_size
            objectCenter[1] = objectCenter[1]/im_hight_size

        return objectCenter


def get_socket_and_send_ini_message(host_adress):
    context = zmq.Context()
    #Socket to talk to server
    print("Connecting to hello world server...")
    socket = context.socket(zmq.REQ)
    socket.connect(host_address)
    socket.send(b"hello, this is kudos_vision!")
    return socket

def open_server_and_wait_for_receive_message(host_adress):
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:9011")
    print("wait receive message from client...")
    message = socket.recv()
    print("message: ", message)

    return socket


if __name__ == '__main__':
    socket = get_socket_and_send_ini_message(host_address)
    socket2 = open_server_and_wait_for_receive_message("tcp://*:9011")
    print("all server open done!")
    DataFormatTransfer = DataFormatTransfer()
    darknet_config_args = kudos_darknet.parser()
    kudos_darknet.check_arguments_errors(darknet_config_args)
    darknet_network, darknet_class_names, darknet_class_colors, darknet_width, darknet_height = kudos_darknet.Initialize_darknet(darknet_config_args)
    cap = cv2.VideoCapture(0)
    for request in range(100):
        frame = znp.recv_array(socket)
        ret = True
        #ret, frame = cap.read()
        ballCenter = [-1.0, -1.0]
        frame, detections = kudos_darknet.getResults_with_darknet(ret, frame, darknet_width, darknet_height, darknet_network, darknet_class_names, darknet_class_colors,darknet_config_args)
        cv2.imshow("showIMG", frame)
        ballCenter = DataFormatTransfer.get_one_center_from_detections(detections, label='ball')
        k = cv2.waitKey(1) 
        if k == 27:
            break
        omo_send_list = [0.666]
        omo_send_list = np.array(omo_send_list)
        znp.send_array(socket, omo_send_list)

        omo_send_list = ballCenter
        omo_send_list = np.array(omo_send_list)
        znp.send_array(socket2, omo_send_list)
        received_npArr = znp.recv_array(socket2)

