#!/usr/bin/env python
# coding: utf-8
​
import rospy
import sys
#sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
​
class colorTracking:
    def __init__(self, ini_cap):
        self.x = 0
        self.y = 0
        self.cap = ini_cap
        self.node_name = "color_tracking"
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("input_imge", Image, self.image_callback, queue_size=1)
        self.pub1 = rospy.Publisher("position", Int32MultiArray, queue_size=1)
        self.pub2 = rospy.Publisher("processedImage", Image, queue_size=2)
​
    def convertColor(self, frame):
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        #しきい値の設定
        hsv_min = np.array([5,100,0])
        hsv_max = np.array([20,255,255])
        mask = cv2.inRange(hsv,hsv_min,hsv_max)
        #molphology trasnsformation
        kernel = np.ones((6,6),dtype=np.uint8)
        img_d = cv2.dilate(mask, kernel, iterations=8)
    
        return img_d
​
    def maxContourAnalysis(self, mask, frame):
        _,contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for i in contours:
            #輪郭の近似
            approx = cv2.convexHull(i)
            #輪郭に長方形
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        
        if len(rects) > 0:
             rect = max(rects, key=(lambda x: x[2] * x[3]))
             area = rect[2]*rect[3]
             if area > 5000:
                #print(area)
                cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (255, 0, 0), thickness=2)
                rectCenter = rect[0:2] + rect[2:4]//2
                self.x = rectCenter[0] - frame.shape[1]//2
                self.y = rectCenter[1] - frame.shape[0]//2
                cv2.circle(frame,tuple(rectCenter),5,(0,0,255),3)
                cv2.putText(frame," x:"+str(self.x)+" y:"+str(self.y), tuple(rectCenter), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1)
                
        return frame
​
    def cleanup(self):
        cv2.destroyAllWindows()
        self.cap.release()
​
def main():
    #get movie data
    cap = cv2.VideoCapture(2)
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    #fourcc = cv2.VideoWriter_fourcc(*"XVID")
    #dst = cv2.VideoWriter("output/test.avi", fourcc, 30.0, (w,h))
​
    Ctrac = colorTracking(cap)
​
    if cap.isOpened() == False:
        sys.exit()
​
    r = rospy.Rate(100) # 10hz
​
    while not rospy.is_shutdown():
        ret,frame = cap.read()
        if ret == False:
            break
        
        mask = Ctrac.convertColor(frame)
        frame = Ctrac.maxContourAnalysis(mask, frame)
    
        point = [Ctrac.x,Ctrac.y]
        point_forPub = Int32MultiArray(data=point)
        #cv2.imshow("img_g",mask)
        #cv2.imshow("img",frame)
        #dst.write(frame)
        #rospy.loginfo(" x:"+str(point[0])+" y:"+str(point[1]))
        Ctrac.pub1.publish(point_forPub)
        Ctrac.pub2.publish(Ctrac.bridge.cv2_to_imgmsg(frame, "bgr8"))
​
        r.sleep()
        cv2.waitKey(30)
​
if __name__ == "__main__":
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    

