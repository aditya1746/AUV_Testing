#! /usr/bin/env python
import rospy
import cv2
import numpy as np
import glob
import random


def callback(msg):
    
    return
    while (vid.isOpened()):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(classes), 3))

        # Insert here the path of your images
        ret, frame = vid.read()     # Loading image

        img = frame
        height, width, channels = img.shape

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
        # print(indexes)
        font = cv2.FONT_HERSHEY_PLAIN
        x_c = []
        y_c = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]

                # print(x)
                # print(y)
                x1 = x + (w / 2)
                y1 = y + (h / 2)
                x_c.append(x1)
                y_c.append(y1)

                print("centre of flare:")
                print(x1)
                print(y1)
                # print(w)
                # print(h)

                label = str(classes[class_ids[i]])
                color = colors[class_ids[i]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 3, color, 2)

        x_avg = 0
        if len(x_c) != 0:
            x_avg = sum(x_c) / len(x_c)
        y_avg = 0
        if len(y_c) != 0:
            y_avg = sum(y_c) / len(y_c)

        print("centre of all detected flare:")
        print(x_avg)
        print(y_avg)
        
        cv2.circle(img, (int(x_avg),int(y_avg)), 2, (255,0,0),2)

        cv2.namedWindow('Image', cv2.WINDOW_NORMAL)

        # cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()

    cv2.destroyAllWindows()


if __name__=='__main__':

    rospy.init_node('yolo_flare_detection', anonymous = False)
    q = 1

    # Load Yolo
    net = cv2.dnn.readNet("yolov3_training_flare.weights", "yolov3_testing.cfg")

    # Name custom object
    classes = ["flare"]

    # Images path
    # images_path = glob.glob(r"D:\ASME\Training\*.jpg")
    vid = cv2.VideoCapture("hanging_gate.webm")

    # rospy.Subscriber('front_cam', Image, callback)

    while (vid.isOpened()):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        colors = np.random.uniform(0, 255, size=(len(classes), 3))

        # Insert here the path of your images
        ret, frame = vid.read()     # Loading image

        img = frame
        height, width, channels = img.shape

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
        # print(indexes)
        font = cv2.FONT_HERSHEY_PLAIN
        x_c = []
        y_c = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]

                # print(x)
                # print(y)
                x1 = x + (w / 2)
                y1 = y + (h / 2)
                x_c.append(x1)
                y_c.append(y1)

                print("centre of flare:")
                print(x1)
                print(y1)
                # print(w)
                # print(h)

                label = str(classes[class_ids[i]])
                color = colors[class_ids[i]]
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, label, (x, y + 30), font, 3, color, 2)

        x_avg = 0
        if len(x_c) != 0:
            x_avg = sum(x_c) / len(x_c)
        y_avg = 0
        if len(y_c) != 0:
            y_avg = sum(y_c) / len(y_c)

        print("centre of all detected flare:")
        print(x_avg)
        print(y_avg)
        
        cv2.circle(img, (int(x_avg),int(y_avg)), 2, (255,0,0),2)

        # cv2.namedWindow('Image', cv2.WINDOW_NORMAL)

        # cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    vid.release()

    cv2.destroyAllWindows()
