#!/usr/bin/env python3  
import random
import rospy
from std_msgs.msg import Int16
from two_int_listener.msg import TwoInts

# Initialize the publisher
def talker():
    # Initialize the published topic 'two_ints'
    pub = rospy.Publisher('two_ints', TwoInts, queue_size=10)
    # Initialize the node two_int_talker'
    rospy.init_node('two_int_talker', anonymous=True)
    rate = rospy.Rate(10)  
    random.seed()

    while not rospy.is_shutdown():
        # Initialize the message
        msg = TwoInts()
        # Generate random integers
        msg.a = random.randint(1,20)
        msg.b = random.randint(1,20)
        # Log the message
        rospy.loginfo('Publishing: %i + %i', msg.a, msg.b)
        rate = rospy.Rate(1)  
        # Publish the message
        pub.publish(msg)
        rate.sleep()    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

