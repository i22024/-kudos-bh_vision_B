"#!/usr/bin/env python"
import cv2
import numpy as np
import darknet
import rospy
from darkneta.msg import position
class priROS():
    def talker(self, posX, posY):
        pub = rospy.Publisher('visionPos', position, queue_size=10)
        rospy.init_node('visionPos', anonymous = False)
        message = position()
        message.posX = posX
        message.posY = posY
        rospy.loginfo(message)
        pub.publish(message)

if __name__=='__main__':
    priROS = priROS()
    while True:
        x =  np.random.randn(10,100)
        posX = 100
        posY = 50
        priROS.talker(posX, posY)