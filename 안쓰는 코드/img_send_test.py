import time
import numpy as np
import rospy
import roslib
from sensor_msgs.msg import CompressedImage
from scipy.ndimage import filters
try:
    import sys
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
    import cv2
except Exception as e:
    pass

class image_feature:
    def __init__(self):
        rospy.init_node('image_feature', anonymous=False)
        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size = 1)
        #self.subscriber = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.callback,  queue_size = 1)

    def send_data(self):
        '''
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2. CV_LOAD_IMAGE_COLOR)
        method = "GridFAST"
        feat_det = cv2.FeatureDetector_create(method)
        time1 = time.time()
        '''

        image_np = cv2.imread('test_img.jpg', cv2.IMREAD_COLOR)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        self.image_pub.publish(msg)

if __name__ == '__main__':
    ic = image_feature()
    while(1):
        ic.send_data()


