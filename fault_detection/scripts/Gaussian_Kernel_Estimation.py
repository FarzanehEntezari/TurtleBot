#! /usr/bin/env python

import rospy
import time
import pickle
import numpy as np
from sensor_msgs.msg import Imu
from sklearn.neighbors import KernelDensity
from sklearn.model_selection import GridSearchCV
import matplotlib.pyplot as plt

class Gaussian_Estimation():
    def __init__ (self):
        #self.kde = KernelDensity(kernel='gaussian' , bandwidth = 0.09 )
        self.kde =GridSearchCV(KernelDensity(kernel='gaussian'),{'bandwidth': np.linspace(0.1,1.5,15)},cv=[(slice(None),slice(None))])
        self.acc_record = []

    def callback(self , msg):
        acc_x = msg.linear_acceleration.x
        acc_z = msg.linear_acceleration.z
        self.acc_record.append ( [acc_x, acc_z] )

    def fitGK(self):
        rospy.loginfo("WAIT: fitting the data is in process!")
        self.kde.fit( np.array(self.acc_record))
        print(self.kde.best_params_)
        self.kde = self.kde.best_estimator_

    def kde_plot(self):
        acc = np.array(self.acc_record)
        minx = np.min(acc[:,0])
        maxx = np.max(acc[:,0])
        minz = np.min(acc[:,1])
        maxz = np.max(acc[:,1])
        #rospy.loginfo(minimum)
        x = np.arange(minx*1.2, maxx*1.2, 0.05)
        z = np.arange(minz*3, maxz*3, 0.05)
        xx, zz = np.meshgrid(x, z, sparse = False)
        xz_samples = np.vstack([xx.ravel(), zz.ravel()]).T
        likelihood =np.atleast_2d( np.exp(self.kde.score_samples(xz_samples)) )
        #likelihood =np.atleast_2d(self.kde.score_samples(xz_samples))
        #rospy.loginfo(xx.shape)
        #print(likelihood.shape)
        likelihood=likelihood.reshape(xx.shape)
        #print(likelihood.shape)
        plt.figure(figsize=(30,15))
        plt.pcolormesh(xx,zz,likelihood)
        plt.clim(  np.min(likelihood),np.max(likelihood)  )
        plt.colorbar()
        plt.scatter(acc[::3,0],acc[::3,1], s=2 , facecolor = 'white')
        plt.xlabel('acceleration in x axis')
        plt.ylabel('acceleration in z axis')
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

