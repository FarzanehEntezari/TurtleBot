#! /usr/bin/env python

import rospy
import time
import pickle
import numpy as np
from sensor_msgs.msg import Imu
from sklearn.neighbors import KernelDensity

class Gaussian_Estimation():
    def __init__ (self):
        self.kde = KernelDensity(kernel='gaussian' , bandwidth = 0.2)
        self.acc_record = []

    def callback(self , msg):
        acc_x = msg.linear_acceleration.x
        acc_z = msg.linear_acceleration.z
        self.acc_record.append ( (acc_x, acc_z) )

    def fitGK(self):
        self.kde.fit(self.acc_record)

def main():
    rospy.init_node('Gaussian_Kernel_Estimation', anonymous = True)
    GK = Gaussian_Estimation()
    rospy.Subscriber('imu' , Imu , GK.callback)
    rospy.spin()
    rospy.loginfo("Interuppted by user!")
    GK.fitGK()
    with open( "GaussianKernel_NormalMode.pickle" , 'wb' ) as pickle_file:
        pickle.dump(GK.kde , pickle_file)
    rospy.loginfo('Saved the estimated Gaussian kernel in a pickle file!')

    #x_plot = np.linspace(-1 , 1 , num=50)
    #z_plot = np.linspace(-1 , 1 , num=50)
    #GK.score_samples()


if  __name__ =="__main__" :
    main()

