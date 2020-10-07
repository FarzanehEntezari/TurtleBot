#!/usr/bin/env python

import rospy
import location_monitor.srv

def main():
	rospy.init_node("landmark_monitor")
	#rospy.spin() #no need to add spin bcz it's not supposed to loop forever

	get_closest = rospy.ServiceProxy("get_closest" , location_monitor.srv.GetClosest)
	get_distance = rospy.ServiceProxy("get_distance" , location_monitor.srv.GetDistance)

	try:
		rospy.wait_for_service('get_closest' , timeout = 10)
		rospy.wait_for_service('get_distance' , timeout = 10)
	except rospy.ROSException :
		rospy.logerr("Services did not respond in 10 seconds!")
		return


	request = location_monitor.srv.GetClosestRequest()
	response = get_closest(request)
	print ("Closest: {}".format(response.name))

	landmarks = ['pile1', 'pile2','pile3','pile4','pile5', 'pile6','pile7','pile8','pile9']
	for landmark in landmarks:
		request = location_monitor.srv.GetDistanceRequest()
		request.name= landmark
		response = get_distance(request)
		print ("{} : {}".format (landmark , response.distance) )

if __name__ == "__main__" :
	main()
