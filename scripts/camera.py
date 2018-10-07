#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('alphabot2')
import rospy
import rospkg
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import cv2
import numpy as np

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('alphabot2')

face_cascade = cv2.CascadeClassifier(os.path.join(
    pkg_path, 'haar/haarcascade_frontalface_default.xml'))
eye_cascade = cv2.CascadeClassifier(
    os.path.join(pkg_path, 'haar/haarcascade_eye.xml'))

cap = cv2.VideoCapture(0)
for i in range(18):
    print(i, cap.get(i))
RESOLUTION = 4
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640 / RESOLUTION)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480 / RESOLUTION)


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("rpi_cam", Image, queue_size=1)
        self.facepos_pub = rospy.Publisher("face_pos", Point, queue_size=1)

        self.bridge = CvBridge()
        while not rospy.core.is_shutdown():
            # Capture frame-by-frame
            ret, cv_image = cap.read()

            if not ret:
                print("Can't record")
                break

            grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(grey, 1.3, 5)

            if len(faces) > 0:

                face = faces[0]  # faces[np.argmax(sizes)]
                x, y, w, h = face
                center_x = x + w / 2
                center_y = y + h / 2

                img_h, img_w = grey.shape[:2]
                print("img w={} h={}".format(img_w, img_h))
                face_pos = Point()
                face_pos.x = (float(center_x) / float(img_w) - 0.5) * 2
                face_pos.y = (float(center_y) / float(img_h) - 0.5) * 2
                self.facepos_pub.publish(face_pos)

                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)

                (rows, cols, channels) = cv_image.shape
                if cols > center_x+10 and rows > center_y+10:
                    cv2.circle(cv_image, (center_x, center_y), 10, 255)

            try:
                print('publish image')
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)


def main(args):
    rospy.init_node('camera', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
