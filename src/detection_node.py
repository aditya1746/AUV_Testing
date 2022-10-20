#! /usr/bin/env python

import cv2
import numpy as np
import glob
import random
import math
from numpy import float32
import rospy
from dynamic_reconfigure.server import Server
#from auv_codes2.cfg import thrusterConfig
# from auv_codes2.cfg import PID_yawConfig
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String




def detect():

    # Load Yolo
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3tiny.cfg")

    # Name custom object
    classes = ["flare"]

    # Images path
    # images_path = glob.glob(r"D:\ASME\Training\*.jpg")
    vid = cv2.VideoCapture(0)

    
    def updateFront(h):
        distance=h/(2*math.tan(0.303))
        frontDistance=(distance*36.5)/h
        return frontDistance

    while (True):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(classes), 3))

        # Insert here the path of your images
        ret, frame = vid.read()     # Loading image

        img = frame
        height, width, channels = img.shape
        horizontalDistance=0


        # Detecting objects
        blob = cv2.dnn.blobFromImage(img, 0.00392, (416, 416), (0, 0, 0), True, crop=False)

        net.setInput(blob)
        outs = net.forward(output_layers)

        # Showing informations on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.2:
                    # Object detected
                    # print(class_id)
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    # x & y are left top coordinates of rectangle
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        if len(indexes)> 0 :
            # print(indexes)
            font = cv2.FONT_HERSHEY_PLAIN
            x_c = []
            y_c = []
            h_c = []
            for i in range(len(boxes)):
                if i in indexes:
                    x, y, w, h = boxes[i]

                    # print(x)
                    # print
                    x1 = x + (w / 2)
                    y1 = y + (h / 2)
                    x_c.append(x1)
                    y_c.append(y1)
                    h_c.append(h)

                    # print("centre of flare:")
                    # print(x1)
                    # print(y1)
                    # print(w)
                    # print(h)

                    label = str(classes[class_ids[i]])
                    color = colors[class_ids[i]]
                    cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                    cv2.putText(img, "gate", (x, y + 30), font, 3, color, 2)

            x_avg = 0
            if len(x_c) != 0:
                x_avg = sum(x_c) / len(x_c)
            y_avg = 0
            if len(y_c) != 0:
                y_avg = sum(y_c) / len(y_c)
            h_avg = 0
            if len(h_c) !=0:
                h_avg= sum(h_c) / len(h_c)

            print("centre of all detected flare:")
            print(x_avg)
            print(y_avg)


            horizontalDistance= x_avg - width/2



            if len(y_c)>1:
                cv2.circle(img,(int(x_avg),int(y_avg)), 2, (0,0,255), 2)
                cv2.putText(img, "centre", (int(x_avg),int(y_avg)), font, 3, color, 2)

        # cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
        pub.publish(horizontalDistance)
        print("horizontalDistance: ",horizontalDistance)
        # cv2.imshow("Image", img)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    vid.release()

    cv2.destroyAllWindows()

if __name__ == "__main__":

    rospy.init_node("detection_node", anonymous = False)
    q=1
    pub=rospy.Publisher("object_distance",Float64 ,queue_size=q)

    detect()

    rospy.spin()
