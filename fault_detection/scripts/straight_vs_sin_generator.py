#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import math
#import random
from fault_detection.msg import Fault


class walk():
    def __init__ (self , pub_cmd , pub_fault , turtlebot3_model):
        self.pub_cmd = pub_cmd
        self.pub_fault = pub_fault
        self.turtlebot3_model = turtlebot3_model
        self.vel_lin_max =0
        self.vel_lin_min =0
        self.vel_ang_max =0
        self.vel_ang_min =0

    def straight_walk(self, duration):
        time_end = time.time() + duration
        
        mode_stwalk = Fault()
        mode_stwalk.mode = "straight_walk"
        mode_stwalk.probability = 1

        twist_stwalk = Twist()
        twist_stwalk.linear.x = 0.25
        twist_stwalk.angular.z =  0

        mode_stop = Fault()
        mode_stop.mode = "stop"
        mode_stop.probability = 1

        twist_stop = Twist()
        twist_stop.linear.x = 0
        twist_stop.angular.z =  0
        rospy.loginfo("Robot is making a straight walk.")
        while True :
            if(time.time() <= time_end and not rospy.is_shutdown() ):
                self.pub_fault.publish(mode_stwalk)
                self.pub_cmd.publish(twist_stwalk)
            else:
                rospy.loginfo("Timeout. Stopping robot!")
                self.pub_fault.publish(mode_stop)
                self.pub_cmd.publish(twist_stop)
                break

    def sin_walk(self, duration):
        time_end = time.time() + duration
        
        mode_sinwalk = Fault()
        mode_sinwalk.mode = "sin_walk"
        mode_sinwalk.probability = 1

        twist_sinwalk = Twist()
        twist_sinwalk.linear.x = 0.25
        twist_sinwalk.angular.z =  0

        mode_stop = Fault()
        mode_stop.mode = "stop"
        mode_stop.probability = 1

        twist_stop = Twist()
        twist_stop.linear.x = 0
        twist_stop.angular.z =  0

        rospy.loginfo("Robot is making a sin walk.")
        while True :
            if(time.time() <= time_end and not rospy.is_shutdown() ):
                twist_sinwalk.angular.z = math.sin(time.time())
                self.pub_fault.publish(mode_sinwalk)
                self.pub_cmd.publish(twist_sinwalk)
            else:
                rospy.loginfo("Timeout. Stopping robot!")
                self.pub_fault.publish(mode_stop)
                self.pub_cmd.publish(twist_stop)
                break

    def terminate(self):
        rospy.loginfo("System is shutting down. Stopping robot...")
        mode_NA = Fault()
        mode_NA.mode = "NA"
        mode_NA.probability = 1

        twist_stop = Twist()
        twist_stop.linear.x = 0
        twist_stop.angular.z =  0
        self.pub_fault.publish(mode_NA)
        self.pub_cmd.publish(twist_stop)

    def set_margin(self):
        BURGER_MAX_LIN_VEL = 0.22
        BURGER_MAX_ANG_VEL = 2.84
        WAFFLE_MAX_LIN_VEL = 0.26
        WAFFLE_MAX_ANG_VEL = 1.82
        LIN_VEL_STEP_SIZE = 0.01
        ANG_VEL_STEP_SIZE = 0.1

        if(self.turtlebot3_model == "burger"):
            self.vel_lin_max =   BURGER_MAX_LIN_VEL
            self.vel_lin_min = - BURGER_MAX_LIN_VEL
            self.vel_ang_max =   BURGER_MAX_ANG_VEL
            self.vel_ang_min = - BURGER_MAX_ANG_VEL
        elif(self.turtlebot3_model == "waffle" | turtlebot3_model == "waffle_pi"):
            self.vel_lin_max =   WAFFLE_MAX_LIN_VEL
            self.vel_lin_min = - WAFFLE_MAX_LIN_VEL
            self.vel_ang_max =   WAFFLE_MAX_ANG_VEL
            self.vel_ang_min = - WAFFLE_MAX_ANG_VEL

def main():
    rospy.init_node("walk_modes_generator", anonymous=True)
    pub_cmd = rospy.Publisher("/cmd_vel" , Twist, queue_size=1 , latch=False)
    pub_fault = rospy.Publisher("/fault_mode", Fault , queue_size=1 , latch=False)
    turtlebot3_model = rospy.get_param("model" , "burger")
    walkwalk = walk(pub_cmd, pub_fault, turtlebot3_model)
    rospy.on_shutdown(walkwalk.terminate)

    #while not rospy.is_shutdown():
    #walkwalk.straight_walk(duration=5)
    walkwalk.sin_walk(duration=50)

if __name__ == "__main__":
    main()