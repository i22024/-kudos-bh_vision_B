import rospy
from darknetb.msg import kudos_vision_gcm
import time
from multiprocessing import Process, Queue

mode = 0

class priROS():
    def __init__(self):
        rospy.init_node('kudos_vision_Game_controller_processor', anonymous = False)

    def talker(self, message_form):
        pub = rospy.Publisher('kudos_vision_gcm', kudos_vision_gcm, queue_size=1)
        message = kudos_vision_gcm()
        message.main_game_state = message_form['main_game_state']
        message.vision_go_local = message_form["vision_go_local_mode"]
        # rospy.loginfo(message)
        pub.publish(message)

def read_file():
    f = open("./bh_function/bh_data/fake_kudos_gcm_text.txt", 'r')
    line = f.readline()
    input_mode = int(line)
    print(input_mode)
    return input_mode

if __name__=='__main__':
    priROS = priROS()
    while True:
        input_mode = read_file()
        message_form = {
            'main_game_state':-1,
            'vision_go_local_mode':-1,
        }
        if input_mode == 0:
            message_form["vision_go_local_mode"] = -1
            message_form["main_game_state"] = -1
        elif input_mode == 1:
            message_form["vision_go_local_mode"] = 1
            message_form["main_game_state"] = -1
        time.sleep(1)
        priROS.talker(message_form)


        
            



