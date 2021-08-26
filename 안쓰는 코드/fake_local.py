import rospy
from darknetb.msg import kudos_vision_local_sensor_data as kvlsd
import time

class priROS():
    def __init__(self):
        rospy.init_node('kudos_vision_local_process', anonymous = False)
    def talker(self, message_form):
        pub = rospy.Publisher('kudos_vision_local_sensor_data', kvlsd, queue_size=1)
        message = kvlsd()
        message.debug_num = message_form['debug_num']
        message.sensor_data_x = message_form['sensor_data_x']
        message.sensor_data_y = message_form['sensor_data_y']
        message.op3_local_mode = message_form['op3_local_mode']
        message.start_point_x = message_form['start_point_x']
        message.start_point_y = message_form['start_point_y']
        message.start_point_orien = message_form['start_point_orien']
        pub.publish(message)

if __name__=='__main__':
    priROS = priROS()
    while True:
        mcl_message = {"debug_num":1,
                    "sensor_data_x":[],
                    "sensor_data_y":[],
                    "op3_local_mode": True,
                    "start_point_x":0,
                    "start_point_y":0,
                    "start_point_orien":0}
        mcl_message["sensor_data_y"].append(0)
        mcl_message["sensor_data_x"].append(100)
        for i in range(99):
            mcl_message["sensor_data_x"].append(-100)
            mcl_message["sensor_data_y"].append(-100)
        priROS.talker(mcl_message)
        time.sleep(5)

        mcl_message = {"debug_num":1,
                    "sensor_data_x":[],
                    "sensor_data_y":[],
                    "op3_local_mode": True,
                    "start_point_x":-300,
                    "start_point_y":0,
                    "start_point_orien":0}
        mcl_message["sensor_data_y"].append(50)
        mcl_message["sensor_data_x"].append(100)
        for i in range(99):
            mcl_message["sensor_data_x"].append(-100)
            mcl_message["sensor_data_y"].append(-100)
        priROS.talker(mcl_message)
        time.sleep(5)

        mcl_message = {"debug_num":1,
                    "sensor_data_x":[],
                    "sensor_data_y":[],
                    "op3_local_mode": True,
                    "start_point_x":300,
                    "start_point_y":0,
                    "start_point_orien":-180}
        mcl_message["sensor_data_y"].append(-50)
        mcl_message["sensor_data_x"].append(100)
        for i in range(99):
            mcl_message["sensor_data_x"].append(-100)
            mcl_message["sensor_data_y"].append(-100)
        priROS.talker(mcl_message)
        time.sleep(5)


        