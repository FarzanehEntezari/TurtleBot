#! /usr/bin/env python

import rospy
import time
import pickle
import numpy as np
from sensor_msgs.msg import Imu
from sklearn.neighbors import KernelDensity
import matplotlib.pyplot as plt

class Gaussian_Estimation():
    def __init__ (self):
        self.kde = KernelDensity(kernel='gaussian' , bandwidth = 0.2)
        self.acc_record = []

    def callback(self , msg):
        acc_x = msg.linear_acceleration.x
        acc_z = msg.linear_acceleration.z
        self.acc_record.append ( [acc_x, acc_z] )

    def fitGK(self):
        self.kde.fit(self.acc_record)

    def kde_plot(self):
        acc = np.array(self.acc_record)
        minimum = np.min(acc)
        maximum = np.max(acc)
        rospy.loginfo(minimum)
        x = np.arange(minimum, maximum, 0.05)
        z = np.arange(minimum, maximum, 0.05)
        xx, zz = np.meshgrid(x, z, sparse = False)
        xz_samples = np.vstack([xx.ravel(), zz.ravel()]).T
        probability =np.atleast_2d( np.exp(self.kde.score_samples(xz_samples)) )
        rospy.loginfo(xx.shape)
        print(probability.shape)
        probability=probability.reshape(xx.shape)
        print(probability.shape)
        plt.pcolormesh(xx,zz,probability)
        #plt.scatter(acc[:,0],acc[:,1],s=2 , facecolor = 'white')
        plt.savefig('kernel.png')

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
    GK.kde_plot()
    rospy.loginfo('Saved a 2d plot of Gaussian kernel')


if  __name__ =="__main__" :
    main()

