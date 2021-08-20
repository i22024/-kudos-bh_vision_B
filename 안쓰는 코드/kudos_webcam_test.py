import cv2 
import time

cap= cv2.VideoCapture(-1)
prev_time = 0
FPS = 10
while True:
    ret, frame = cap.read()
    cv2.imshow('VideoCapture', frame)
    k = cv2.waitKey(1) 
    if k == 27:
        break
cap.release() 
cv2.destroyAllWindows()