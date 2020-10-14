#!/usr/bin/python

import rospy
import time
import actionlib
from location_monitor.msg import timerAction, timerGoal, timerResult, timerFeedback

def feedback_callback(feedback):
	rospy.loginfo("[Feedback] Time elapsed %f" %(feedback.time_elapsed.to_sec() ) )
	rospy.loginfo("[Feedback] Time remaining %f" %(feedback.time_reamining.to_sec() ) )

def main():
	rospy.init_node('timer_action_client')
	client = actionlib.SimpleActionClient('timer', timerAction)
	client.wait_for_server()
	goal = timerGoal()
	#goal.time_to_wait = rospy.Duration.from_sec(20.0)
	goal.time_to_wait = rospy.Duration.from_sec(500.0)

	client.send_goal(goal , feedback_cb = feedback_callback)

	#time.sleep(3.0)
	#client.cancel_goal()

	client.wait_for_result()

	print('[Result] State: %d ' %(client.get_state() ) )
	print('[Result] Status: %d ' %(client.get_goal_status_text() ) )
	if client.get_result() :
		print ('[Result] time elapsed : %f' %( client.get_result().time_elapsed.to_sec() ) )
		print ('[Result] updates sent : %f' %( client.get_result().updates_sent ) )

if __name__ ==" __main__" :
	main()
