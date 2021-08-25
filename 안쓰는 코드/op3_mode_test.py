import time
import rospy
from std_msgs.msg import Bool as rosBool

class priROS():
    def __init__(self):
        rospy.init_node('fake_op3', anonymous = False)

    def talker(self, local_mode):
        pub = rospy.Publisher('kudos_vision_op3_local_mode_msg', rosBool, queue_size=1)
        pub.publish(local_mode)

if __name__=='__main__':
    priROS = priROS()
    while True:
        priROS.talker(True)
        print("talk True")
        time.sleep(4)
        priROS.talker(False)
        print("talk False")
        time.sleep(4)

