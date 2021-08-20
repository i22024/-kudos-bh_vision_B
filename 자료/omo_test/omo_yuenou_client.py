import zmq
import zmqnumpy as znp
import numpy as np
import kudos_darknet
import time
import cv2


host_address = "tcp://localhost:9011"

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
    socket.send(b"hello, this is ym_com!")
    return socket

if __name__ == '__main__':
    socket = get_socket_and_send_ini_message(host_address)
    while 1:
        frame = znp.recv_array(socket)
        print(frame)
        omo_send_list = [0.666]
        omo_send_list = np.array(omo_send_list)
        znp.send_array(socket, omo_send_list)