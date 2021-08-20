import zmq
import zmqnumpy as znp
import numpy as np
import time
import cv2

#robot Server!
if __name__ == '__main__':
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:9010")
    print("wait receive message from client...")
    message = socket.recv()
    print("message: ", message)
    cap = cv2.VideoCapture(0)
    while 1:
        ret, frame = cap.read()
        np.shape(frame)
        if np.any(frame) != False:
            znp.send_array(socket, frame)
            
        received_npArr = znp.recv_array(socket)

