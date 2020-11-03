#! /usr/bin/env python

import rospy
import time
import pickle
import numpy as np
from sensor_msgs.msg import Imu
from fault_detection.msg import Fault
from sklearn.neighbors import KernelDensity
from sklearn.model_selection import GridSearchCV
import matplotlib.pyplot as plt

class Gaussian_Estimation():
    def __init__ (self):
        self.kde = KernelDensity(kernel='gaussian' , bandwidth = 0.09 )
        #self.kde =GridSearchCV(KernelDensity(kernel='gaussian'),{'bandwidth': np.linspace(0.1,1.5,15)},cv=[(slice(None),slice(None))])
        self.acc_record = []
        self.kde_f = KernelDensity(kernel='gaussian' , bandwidth = 0.09 )
        #self.kde_f =GridSearchCV(KernelDensity(kernel='gaussian'),{'bandwidth': np.linspace(0.1,1.5,15)},cv=[(slice(None),slice(None))])
        self.acc_record_f = []
        self.mode = 0

    def callback_mode(self, msg):
        if (msg.mode == "Normal" ):
            self.mode = 0
        elif(msg.mode == "fault_cmdvel"):
            self.mode = 1
        else:
            rospy.logwarn("recieved undefined fault mode! ")

    def callback(self , msg):
        if (self.mode == 0 ):
            acc_x = msg.linear_acceleration.x
            acc_z = msg.linear_acceleration.z
            self.acc_record.append ( [acc_x, acc_z] )
        if (self.mode == 1 ):
            acc_x = msg.linear_acceleration.x
            acc_z = msg.linear_acceleration.z
            self.acc_record_f.append ( [acc_x, acc_z] )

    def fitGK(self):
        rospy.loginfo("WAIT: fitting the data is in process!")
        self.kde.fit( np.array(self.acc_record))
        #print(self.kde.best_params_)
        #self.kde = self.kde.best_estimator_
        self.kde_f.fit( np.array(self.acc_record_f))
        #print(self.kde_f.best_params_)
        #self.kde_f = self.kde_f.best_estimator_

    def kde_plot(self , mode):
        if (mode == "Normal"):
            acc = np.array(self.acc_record)
            kd = self.kde
            name = "normal_mode_kernel"
        elif(mode == "fault"):
            acc = np.array(self.acc_record_f)
            kd = self.kde_f
            name = "fault_mode_kernel"
        else:
            rospy.logwarn("undefined mode!")

        #acc = np.array(self.acc_record)
        minx = np.min(acc[:,0])
        maxx = np.max(acc[:,0])
        minz = np.min(acc[:,1])
        maxz = np.max(acc[:,1])
        #rospy.loginfo(minimum)
        x = np.arange(minx*1.2, maxx*1.2, 0.05)
        z = np.arange(minz*3, maxz*3, 0.05)
        xx, zz = np.meshgrid(x, z, sparse = False)
        xz_samples = np.vstack([xx.ravel(), zz.ravel()]).T
        #likelihood =np.atleast_2d( np.exp(self.kde.score_samples(xz_samples)) )
        likelihood =np.atleast_2d( np.exp(kd.score_samples(xz_samples)) )
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
        plt.savefig(name+".png")

def main():
    rospy.init_node('Gaussian_Kernel_Estimation', anonymous = True)
    GK = Gaussian_Estimation()
    rospy.Subscriber('imu' , Imu , GK.callback)
    rospy.Subscriber('fault_mode' , Fault , GK.callback_mode)
    rospy.spin()
    rospy.loginfo("Interuppted by user!")
    GK.fitGK()
    with open( "GaussianKernel_NormalMode.pickle" , 'wb' ) as pickle_file:
        pickle.dump(GK.kde , pickle_file)
    with open( "GaussianKernel_FaultMode.pickle" , 'wb' ) as pickle_file:
        pickle.dump(GK.kde_f , pickle_file)
    rospy.loginfo('Saved the estimated Gaussian kernels in pickle files!')
    GK.kde_plot("Normal")
    GK.kde_plot("fault")
    rospy.loginfo('Saved a 2d plot of Gaussian kernel')


if  __name__ =="__main__" :
    main()

