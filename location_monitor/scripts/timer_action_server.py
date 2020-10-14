#!/usr/bin/python
import rospy
import time
import actionlib
from location_monitor.msg import timerAction, timerGoal, timerFeedback, timerResult

def timer_callback(goal):
	start_time = time.time()
	update_count = 0
	if goal.time_to_wait.to_sec() > 60 :
		result = timerResult()
		result.time_elapsed =rospy.Duration.from_sec( time.time() - start_time )
		result.updates_sent = update_count
		server.set_aborted(result , "Aborted: Too long to wait!")
		return
	while ( time.time() - start_time < goal.time_to_wait.to_sec()  ):
		if server.is_preemt_requested() :
			result = timerResult()
			result.time_elapsed =rospy.Duration.from_sec( time.time() - start_time )
			result.updates_sent = update_count
			server.set_preemted(result , "Timer preemted by user's request!")
			return
		feedback = timerFeedback()
		feedback.time_elapsed = rospy.Duration.from_sec( time.time() - start_time )
		feedback.time_remaining = goal.time_to_wait - feedback.time_elapsed
		server.publish_feedback(feedback)
		update_count += 1
		time.sleep(1.0)
		result = timerResult()
		result.time_elapsed =rospy.Duration.from_sec( time.time() - start_time )
		result.updates_sent = update_count
		server.set_succeeded(result , "Timer completed!")

def main():
	rospy.init_node("timer_action_server")
	server = actionlib.SimpleActionServer("timer" , timerAction , timer_callback , False)
	server.start()
	rospy.spin()

if __name__ == "__main__":
	main()

