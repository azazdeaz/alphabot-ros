#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('alphabot2')
import rospy
import rospkg
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from alphabot2.msg import Circle, AprilTag
from cv_bridge import CvBridge, CvBridgeError

import sys
import os
import cv2
import apriltag
import numpy as np

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('alphabot2')

face_cascade = cv2.CascadeClassifier(os.path.join(
    pkg_path, 'haar/haarcascade_frontalface_default.xml'))
eye_cascade = cv2.CascadeClassifier(
    os.path.join(pkg_path, 'haar/haarcascade_eye.xml'))

cap = cv2.VideoCapture(0)
print('WB', cap.get(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U))
cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 0.0)

for i in range(18):
    print(i, cap.get(i))
RESOLUTION = 4

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640 / RESOLUTION)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480 / RESOLUTION)
cap.set(cv2.CAP_PROP_CONTRAST, 100)
cap.set(cv2.CAP_PROP_SATURATION, 50)


def adjust_gamma(image, gamma=1.0):
	# build a lookup table mapping the pixel values [0, 255] to
	# their adjusted gamma values
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
                   for i in np.arange(0, 256)]).astype("uint8")

	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)


class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("rpi_cam", Image, queue_size=1)
        self.image_debug1_pub = rospy.Publisher("rpi_cam_debug1", Image, queue_size=1)
        self.image_debug2_pub = rospy.Publisher("rpi_cam_debug2", Image, queue_size=1)
        self.image_debug3_pub = rospy.Publisher("rpi_cam_debug3", Image, queue_size=1)
        self.image_debug4_pub = rospy.Publisher("rpi_cam_debug4", Image, queue_size=1)
        self.facepos_pub = rospy.Publisher("face_pos", Point, queue_size=1)
        self.circlepos_pub = rospy.Publisher("circle_pos", Circle, queue_size=1)
        self.apriltag_pub = rospy.Publisher("apriltag", AprilTag, queue_size=1)

        rospy.set_param('canny_th', 200)
        rospy.set_param('param2', 30)
        rospy.set_param('gamma', 1.0)

        self.bridge = CvBridge()
        while not rospy.core.is_shutdown():
            # Capture frame-by-frame
            ret, cv_image = cap.read()

            if not ret:
                print("Can't record")
                break

            cv_image = adjust_gamma(cv_image, rospy.get_param('gamma'))
            grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # self.detect_face(grey, cv_image)
            # self.detect_ball(grey, cv_image)
            self.detect_apriltag(grey, cv_image)

            try:
                self.image_pub.publish(
                    self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

    def detect_face(self, bw_img, img=None):
        faces = face_cascade.detectMultiScale(bw_img, 1.3, 5)

        if len(faces) > 0:

            face = faces[0]  # faces[np.argmax(sizes)]
            x, y, w, h = face
            center_x = x + w / 2
            center_y = y + h / 2

            img_h, img_w = bw_img.shape[:2]
            face_pos = Point()
            face_pos.x = (float(center_x) / float(img_w) - 0.5) * 2
            face_pos.y = (float(center_y) / float(img_h) - 0.5) * 2
            self.facepos_pub.publish(face_pos)

            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

            (rows, cols, channels) = img.shape
            if cols > center_x+10 and rows > center_y+10:
                cv2.circle(img, (center_x, center_y), 10, 255)

    def detect_apriltag(self, bw_img, img=None):
        detector = apriltag.Detector()
        result = detector.detect(bw_img)
        print(result)
        if len(result) < 1:
            return

        tag = result[0]
        msg = AprilTag()
        msg.tag_family = tag.tag_family
        msg.tag_id = tag.tag_id
        msg.hamming = tag.hamming
        msg.decision_margin = tag.decision_margin
        msg.goodness = tag.goodness
        msg.center.x = tag.center[0]
        msg.center.y = tag.center[1]
        msg.corner_a.x = tag.corners[0][0]
        msg.corner_a.y = tag.corners[0][1]
        msg.corner_b.x = tag.corners[1][0]
        msg.corner_b.y = tag.corners[1][1]
        msg.corner_c.x = tag.corners[2][0]
        msg.corner_c.y = tag.corners[2][1]
        msg.corner_d.x = tag.corners[3][0]
        msg.corner_d.y = tag.corners[3][1]
        self.apriltag_pub.publish(msg)

        def pos(xy):
            return (int(xy[0]), int(xy[1]))

        if img is not None:
            for tag in result:
                blue = (0, 0, 255)
                green = (0, 255, 0)

                cv2.line(img, pos(tag.corners[0]), pos(tag.corners[1]), blue, 2)
                cv2.line(img, pos(tag.corners[1]), pos(tag.corners[2]), blue, 2)
                cv2.line(img, pos(tag.corners[2]), pos(tag.corners[3]), blue, 2)
                cv2.line(img, pos(tag.corners[3]), pos(tag.corners[0]), blue, 2)
                cv2.circle(img, pos(tag.center), 12, blue, 2)


    def detect_ball(self, bw_img, img=None):
        faces = face_cascade.detectMultiScale(bw_img, 1.3, 5)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # hsv[:, :, 1] = 255
        hsv[:, :, 2] = 120
        # bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        # gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = hsv[:, :, 1]
        gray = cv2.equalizeHist(gray)

        canny_th = rospy.get_param('canny_th')
        param2 = rospy.get_param('param2')
        edges = cv2.Canny(bw_img, canny_th/2, canny_th)

        img_h, img_w = bw_img.shape[:2]

        ret, thresh = cv2.threshold(bw_img.copy(), 127, 255, 0)
        image, contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (0, 255, 0), 2)

        self.image_debug1_pub.publish(
            self.bridge.cv2_to_imgmsg(edges)
            # self.bridge.cv2_to_imgmsg(bgr, "bgr8")
            # self.bridge.cv2_to_imgmsg(hsv[:, :, 1])
        )
        self.image_debug2_pub.publish(
            self.bridge.cv2_to_imgmsg(gray)
            # self.bridge.cv2_to_imgmsg(bgr, "bgr8")
            # self.bridge.cv2_to_imgmsg(hsv[:, :, 1])
        )
        self.image_debug3_pub.publish(
            self.bridge.cv2_to_imgmsg(thresh)
        )

        return
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=canny_th, param2=param2, minRadius=img_h/20, maxRadius=img_h)

        print('circles', circles)
        if circles is None:
            return

        # print(circles)
        circles = np.uint16(np.around(circles))
        # print(circles)
        if len(circles) > 0:
            circle = circles[0, 0]

            payload = Circle()
            payload.x = circle[0]
            payload.y = circle[1]
            payload.r = circle[2]
            self.circlepos_pub.publish(payload)

            blue = (0, 0, 255)
            green = (0, 255, 0)
            for i in circles[0, :]:
                color = blue
                if i[0] == circle[0]:
                    color = green
                cv2.circle(img, (i[0], i[1]), i[2], color, 2)


def main(args):
    rospy.init_node('camera', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
