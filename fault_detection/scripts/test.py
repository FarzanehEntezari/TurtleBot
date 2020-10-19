#! /usr/bin/env python

import pickle
import numpy as np
from sklearn.neighbors import KernelDensity

#print("Hello!")
#
#
#A= np.array ([0.5,0.5,0.5,0.6,0.52,0.45,0.7,0.3,0.5,0.5] ).reshape(5,2)
#kk = KernelDensity(kernel='gaussian' , bandwidth = 0.2)
#kk.fit(A)
#
#
#kk3 = kk
#print (kk3 == kk)
#
#with open( "G.pickle" , 'wb' ) as file :
#    pickle.dump( kk , file)
#
#
#with open( "GaussianKernel_NormalMode.pickle" , 'rb' ) as file2 :
#    kk2 = pickle.load(file2)
#    print("Hey")
#    print( kk == kk2 )
#    print(kk)
#    print(kk2)
##print(GK.params())
#
#b=np.array( [0.5,0.4] ).reshape(1,2)
#print (kk.score_samples(b) , kk2.score_samples(b) , kk3.score_samples(b) )
#
#
#minimum = min(self.acc_record)
#maximum = max(self.acc_record)
#rospy.loginfo(minimum , maximum)
#x = np.arange(minimum, maximum, 0.1)
#z = np.arange(minimum, maximum, 0.1)
#xx, zz = np.meshgrid(x, y, sparse=True)
#xz_samples == np.vstack([xx.ravel(), zz.ravel()]).T
#probability = np.exp(self.kde.score_samples(xz_samples))
#

ll = [ [1,1],[1,2],[2,5],[3,1]  ]
print(min(ll))
print(max(ll))


a= np.array(ll)

print(np.min(a))
print(np.max(a))

#a.append ( [1,2] )
#a.append ([3,4])
#print(a)
