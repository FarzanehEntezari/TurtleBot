#! /usr/bin/env python


import rospy
import time
from sensor_msgs.msg import Imu
from sklearn.neighbors import KernelDensity

class Gaussian_Estimation():
    def __init__ (self):
        self.kde = KernelDensity(kernel='gaussian' , bandwidth = 0.2)

    def callback(self , msg):
        rospy.loginfo("Got the message!")
        #rospy.loginfo(msg.linear_acceleration.x)
        self.kde.partial_fit(msg.linear_acceleration.x)
        if time.time % 30 == 0 :
            print(self.kde.get_params(deep=False) )


def main():
    rospy.init_node('fault_detection', anonymous = True)
    GK = Gaussian_Estimation()
    rospy.Subscriber('imu' , Imu , GK.callback)
    rospy.spin()

if  __name__ =="__main__" :
    main()
