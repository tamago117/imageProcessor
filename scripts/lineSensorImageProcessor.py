#!/usr/bin/env python
# coding: utf-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray

class lineSensorImageProcessor:
    def __init__(self):
        self.nodeName = "lineSensorImageProcessor"
        rospy.init_node(self.nodeName)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()

        self.arrayPub = rospy.Publisher("processedResult",Float32MultiArray, queue_size=10)
        self.imagePub = rospy.Publisher("processedImage",Image, queue_size= 10)
        self.imageSub = rospy.Subscriber("lineSensorImageProcessor/image", Image, self.image_callback, queue_size=10)

        #r = rospy.Rate(50)

    def image_callback(self, input_image):
        try:
            image = self.bridge.imgmsg_to_cv2(input_image, "mono8")
        except CvBridgeError, e:
            print e
        output_image = self.ImageProcessor(image)
        pubOutput_image = self.bridge.cv2_to_imgmsg(output_image, "mono8")
        
        self.imagePub.publish(pubOutput_image)

    def ImageProcessor(self, image):
        #image_blur = cv2.bilateralFilter(image, 10, 20, 20)
        #image_blur = cv2.medianBlur(image, 9)
        image_blur = cv2.GaussianBlur(image, (31,31), 2)

        #ret,image_th = cv2.threshold(image, 120, 255, cv2.THRESH_BINARY)

        #image_eq = cv2.equalizeHist(image)
        
        return image_blur

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    lineSensorImageProcessor()
    rospy.spin()