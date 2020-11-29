#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def callback(message):
    #rospy.loginfo("recieved %s", message.data)
    rospy.loginfo(" x:"+str(message.data[0])+" y:"+str(message.data[1]))

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("resultImage", Int32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
