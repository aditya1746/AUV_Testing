#! /usr/bin/env python
from numpy import float32
import rospy
from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
# from auv_codes2.cfg import PID_yawConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
# from auv_codes2.msg import pid_straight_msg
import time
from matplotlib import pyplot as plt
import signal, sys
import cv2


rospy.init_node('video_rec', anonymous = False)

cap = cv2.VideoCapture(0)

fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
out = cv2.VideoWriter('output3.avi', fourcc, 20.0, (640, 480))

while(cap.isOpened()):

    ret, frame = cap.read()

    out.write(frame)


cap.release()
cv2.destroyAllWindows()

