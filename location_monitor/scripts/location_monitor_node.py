#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from location_monitor.msg import LandmarkDistance
from location_monitor.srv import (GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse)



def distance(x1,y1,x2,y2):
	dx = x1-x2
	dy = y1-y2
	return math.sqrt(dx*dx + dy*dy)

class LandmarkMonitor(object):
    def __init__(self,pub, landmarks):
	self._pub = pub
	self._landmarks = landmarks
	self._x=0
	self._y=0

    def odom_callback(self , msg):
	self._x = msg.pose.pose.position.x
	self._y = msg.pose.pose.position.y

        #rospy.loginfo( 'x= {} , y= {}'.format(x,y) )
	closest_name , closest_distance = self.cal_closest()
	ld = LandmarkDistance()
	ld.name = closest_name
	ld.distance = closest_distance
	self._pub.publish(ld)
	if (closest_distance < 0.3):
		rospy.loginfo( "I may hit the {}".format(closest_name) )
	#rospy.loginfo( 'closest to {}'.format(closest_name) )

    def cal_closest(self):
	closest_name = None
	closest_distance = None
	for l_name in self._landmarks:
		l_x , l_y = self._landmarks[l_name]
		dist = distance ( self._x, self._y , l_x, l_y)
		if closest_distance is None or dist<closest_distance :
			closest_distance = dist
			closest_name = l_name
	return closest_name , closest_distance


    def getdistance_callback(self, req):
	rospy.loginfo("Get Distance called with {}!".format(req) )
	if req.name not in self._landmarks :
		rospy.logerr("Unknown landmark {}".format(req.name))
		return None
	response = GetDistanceResponse()
	l_x, l_y = self._landmarks[req.name]
	response.distance = distance(l_x , l_y , self._x , self._y)
	return response


    def getclosest_callback(self , req):
	rospy.loginfo("Get Closest called!")
	closest_name , closest_distance = self.cal_closest()
	response = GetClosestResponse()
	response.name = closest_name
	return response



def main():
    rospy.init_node('location_monitor')

    landmarks = {
	"pile1" :( 1 , 1),
	"pile2" :( 1 , 0) ,
	"pile3" :( 1 , -1),
	"pile4" :( 0 , 1) ,
	"pile5" :( 0 , 0) ,
	"pile6" :( 0 , -1),
	"pile7" :( -1 , 1),
	"pile8" :( -1 , 0) ,
	"pile9" :( -1 , -1)
    }

    pub = rospy.Publisher('closest_landmark' , LandmarkDistance , queue_size=10)
    monitor = LandmarkMonitor(pub , landmarks)

    get_closest = rospy.Service('get_closest' , GetClosest , monitor.getclosest_callback)
    get_distance = rospy.Service('get_distance' , GetDistance , monitor.getdistance_callback)

    rospy.Subscriber('/odom', Odometry, monitor.odom_callback)
    rospy.spin()

if __name__ == '__main__' :
    main()

