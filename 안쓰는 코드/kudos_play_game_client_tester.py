import zmq
import zmqnumpy as znp
import numpy as np
import time

host_address = "tcp://192.168.0.20:9010"

context = zmq.Context()

#Socket to talk to server
print("Connecting to hello world server...")
socket = context.socket(zmq.REQ)
socket.connect(host_address)
socket.send(b"hello, this is kudos_play_game_client_tester!")

for request in range(100):
    received_npArr = znp.recv_array(socket)
    print(np.shape(received_npArr))
    time.sleep(0.01)
    position_list = [0.666,0.999,0.444,0.222]
    position_npArr = np.array(position_list)
    znp.send_array(socket, position_npArr)

