#! /usr/bin/env python


import rospy
import pickle
import numpy as np
from sensor_msgs.msg import Imu
from sklearn.neighbors import KernelDensity

class Probabality_Estimate():
    def __init__ (self):
         with open( "GaussianKernel_NormalMode.pickle" , 'rb' ) as file :
             self.kde = pickle.load(file)

    def callback(self , msg):
        acc_x = msg.linear_acceleration.x
        acc_z = msg.linear_acceleration.z
        acc = np.array( [acc_x , acc_z]  ).reshape(1,2)
        logprob = self.kde.score_samples(acc)
        rospy.loginfo( "p of normal: %f" %( np.exp(logprob) ))

def main():
    rospy.init_node('fault_detection', anonymous = True)
    PE = Probabality_Estimate()
    rospy.Subscriber('imu' , Imu , PE.callback)
    rospy.spin()

if  __name__ =="__main__" :
    main()
