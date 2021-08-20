import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except Exception as e:
    pass
import cv2
import matplotlib.pyplot as plt
import numpy as np

img = cv2.imread('./made_data/ex1.jpg')
contour_img = np.empty_like(img)
contour_img[:,:,:] = np.array([0,0,0])
contour_fill_img = np.empty_like(img)
contour_fill_img[:,:,:] = np.array([0,0,0])
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img = cv2.GaussianBlur(img, (5, 5), 0)
plt.imshow(img)
plt.show()
mask_minimum_condition = np.array([20, 230, 20])
mask_maximum_condition = np.array([40, 250, 40])
thresh_img = cv2.inRange(img, mask_minimum_condition, mask_maximum_condition)
plt.imshow(thresh_img)
plt.show()

contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    cv2.drawContours(contour_img, [contour],0, (255, 255, 255),3)
plt.imshow(contour_img)
plt.show()

cv2.fillPoly(contour_fill_img, pts=contours, color=(255,255,255))
plt.imshow(contour_fill_img)
plt.show()

dilate_power = 20
erode_power = 20

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dilate_power, dilate_power))
contour_fill_img = cv2.dilate(contour_fill_img, kernel)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (erode_power, erode_power))
contour_fill_img = cv2.erode(contour_fill_img, kernel)
plt.imshow(contour_fill_img)
plt.show()