#!/usr/bin/env python3
# project 1: solution.py
import rospy

from std_msgs.msg import Int16
from my_listener.msg import TwoInts

pub = rospy.Publisher('sum', Int16, queue_size=10)

def callback(data):
    ### YOUR CODE HERE ###
    sum = data.a + data.b
    print("a(",data.a,") + b(",data.b,") = ",sum) 
    #rospy.loginfo(rospy.get_caller_id() + 'Sum: %i + %i = %i', data.a, data.b, ata.a + data.b)
    #rate = rospy.Rate(5)
    rospy.loginfo(sum)
    rate = rospy.Rate(1)  
    pub.publish(sum)
    rate.sleep()


def listener():
    rospy.init_node('solution')
    rospy.Subscriber("two_ints", TwoInts, callback)

    rospy.spin()

if __name__=="__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

