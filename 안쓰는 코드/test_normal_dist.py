import rospy
import math
import numpy as np
from darknetb.msg import kudos_vision_local_sensor_data as kvlsd

mcl_message = {"debug_num":1,
                "sensor_data_x":[],
                "sensor_data_y":[],
                "op3_local_mode": False,
                "start_point_x":0,
                "start_point_y":0,
                "start_point_orien":0}

start_point_xy_normal_distribution_range = 3
start_point_orien_normal_distribution_range = 45
start_point_particle_num = 100


class priROS():
    def __init__(self):
        rospy.init_node('test_normal_dist', anonymous = False)

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

if __name__ == '__main__':
    priROS = priROS()
    stp_rand_oriens = np.random.uniform(low=-3.17, high=3.17, size=start_point_particle_num)
    stp_rand_distances = np.random.normal(0, start_point_xy_normal_distribution_range, size=start_point_particle_num)
    print(stp_rand_distances)
    for i in range(start_point_particle_num):
        x = math.cos(stp_rand_oriens[i])*stp_rand_distances[i]
        y = math.sin(stp_rand_oriens[i])*stp_rand_distances[i]
        print("{} {}".format(x, y))
    

