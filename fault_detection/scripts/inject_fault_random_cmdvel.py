#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import random
from fault_detection.msg import Fault


class fault_injection():
    def __init__(self, pub, pub_f, turtlebot3_model):
        self.pub = pub
        self.pub_f = pub_f
        self.turtlebot3_model = turtlebot3_model
        self.vel_lin_max =0
        self.vel_lin_min =0
        self.vel_ang_max =0
        self.vel_ang_min =0

    def injection_time(self):
        delay = random.uniform(13,15)
        print("delay = %f" %(delay) )
        start = time.time()+delay
        no_fault = Fault()
        no_fault.mode = "Normal"
        no_fault.probability = 1
        while True :
            self.pub_f.publish(no_fault)
            if (time.time() >= start) :
                duration = random.uniform(10,20)
                break
        self.injection(duration)

    def injection(self ,  duration ):
        print("inject for %f" %(duration))
        timeout = time.time() + duration
        cmdvel_fault = Fault()
        cmdvel_fault.mode = "fault_cmdvel"
        cmdvel_fault.probability = 1
        while (time.time() <= timeout):
            twist = Twist()
            twist.linear.x = random.uniform(self.vel_lin_min,self.vel_lin_max)
            twist.angular.z =  random.uniform(self.vel_ang_min,self.vel_ang_max)
            self.pub.publish(twist)
            self.pub_f.publish(cmdvel_fault)
            #print("published!")
    def set_margin(self):
        BURGER_MAX_LIN_VEL = 0.22
        BURGER_MAX_ANG_VEL = 2.84
        WAFFLE_MAX_LIN_VEL = 0.26
        WAFFLE_MAX_ANG_VEL = 1.82
        LIN_VEL_STEP_SIZE = 0.01
        ANG_VEL_STEP_SIZE = 0.1

        if(self.turtlebot3_model == "burger"):
            self.vel_lin_max = BURGER_MAX_LIN_VEL
            self.vel_lin_min = - BURGER_MAX_LIN_VEL
            self.vel_ang_max =  BURGER_MAX_ANG_VEL
            self.vel_ang_min = - BURGER_MAX_ANG_VEL
        elif(self.turtlebot3_model == "waffle" | turtlebot3_model == "waffle_pi"):
            self.vel_lin_max = WAFFLE_MAX_LIN_VEL
            self.vel_lin_min = - WAFFLE_MAX_LIN_VEL
            self.vel_ang_max =  WAFFLE_MAX_ANG_VEL
            self.vel_ang_min = - WAFFLE_MAX_ANG_VEL


def main():
    rospy.init_node("fault_injection" , anonymous=True)
    pub = rospy.Publisher( "/cmd_vel" , Twist , queue_size =1 , latch=False )
    pub_f = rospy.Publisher( "/fault_mode" , Fault , queue_size =1 , latch=False )
    turtlebot3_model = rospy.get_param("model", "burger")
    fi= fault_injection(pub ,pub_f ,  turtlebot3_model)
    fi.set_margin()
    while not rospy.is_shutdown() :
        fi.injection_time()
    #rospy.spin() not needed bcz we don't have a subscriber!

if __name__ == "__main__":
    main()
