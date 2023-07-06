#%%
import rospy
import time
rospy.init_node('ladofa')

#%%
from sensor_msgs.msg import Image
camera_data = None
def camera_callback(data):
    global camera_data
    camera_data = data

camera_sub = rospy.Subscriber('/mycam', Image, camera_callback) 

# %%
import numpy as np
import cv2

def get_image():
    data = np.frombuffer(camera_data.data, np.uint8)
    H, W = camera_data.height, camera_data.width
    image = data.reshape(H, W, 3)
    return image

from geometry_msgs.msg import Twist
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
forward = Twist()
forward.linear.x = 0.08
left = Twist()
left.angular.z = 0.8
right = Twist()
right.angular.z = -0.8

while camera_data is None:
    time.sleep(1)

import yolo_detector

while True:
    image = get_image()
    det = yolo_detector.detect(image)

    bots = []
    for x1, y1, x2, y2, conf, cat in det:
        if cat != 0:
            continue
        bot = (x2-x1, x1,y1,x2,y2) #x2-x1-> width 이걸 기준으로 정렬할것이다.
        bots.append(bot)
    bots.sort(reverse=True)
    #로봇의 너비를 기준으로 정렬이 됨.
    if len(bots) >  0:
        distance = 320/bots[0][0] #로봇과 로봇사이의 거리
        cx = (bots[0][1] + bots[0][3]) / 2 - 640/2 #로봇의 중심에서 떨어진 거리 #캡처 사진 참고
        
        if distance < 1:
            if cx > 0:
               vel_pub.publish(right)
            else:
                vel_pub.publish(left)
        else:
            if cx > 30:
                vel_pub.publish(right)
            elif cx < -30:
                vel_pub.publish(left)
            else:
                vel_pub.publish(forward)
        #print(bots[0][0], 320/bots[0][0]) #로봇들 중 가장 가까운 로봇을 찾기 tuple 중 첫번째것을 (너비) 츨략
      #c = x*d (d는 로봇 사이의 거리 x는 로봇의 크기) -> c를 계산 가능  C/bots[0][0] -> 거리
      #theata = 
    else:
        vel_pub.publish(left)


    dst = yolo_detector.draw_boxes(image, det)

    cv2.imshow('dst', dst)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break


