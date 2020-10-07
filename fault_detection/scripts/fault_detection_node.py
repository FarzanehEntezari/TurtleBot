#! /usr/bin/env python


import rospy
from sensor_msgs.msg import Imu

def callback(msg):
    rospy.loginfo("Got the message!")

def main():
    rospy.init_node('fault_detection', anonymous = True)
    rospy.Subscriber('imu' , Imu , callback)
    rospy.spin()

if  __name__ =="__main__" :
    main()
