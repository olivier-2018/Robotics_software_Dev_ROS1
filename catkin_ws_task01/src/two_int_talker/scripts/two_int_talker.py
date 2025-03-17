#!/usr/bin/env python3  
# Project 1: two_int_talker.py
import random

import rospy
from std_msgs.msg import Int16
from my_listener.msg import TwoInts

def talker():
    pub = rospy.Publisher('two_ints', TwoInts, queue_size=10)
    rospy.init_node('two_int_talker', anonymous=True)
    rate = rospy.Rate(10)  
    random.seed()

    while not rospy.is_shutdown():

        msg = TwoInts()
  
        msg.a = random.randint(1,20)
        msg.b = random.randint(1,20)
        pub.publish(msg)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



        
