#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from two_int_listener.msg import TwoInts


# Initialize the publisher
pub = rospy.Publisher('sum', Int16, queue_size=10)

def callback(data):
    ## Evaluate the sum of a and b
    sum = data.a + data.b
    #print("a(",data.a,") + b(",data.b,") = ",sum) 
    
    # Log the sum
    rospy.loginfo(rospy.get_caller_id() + ' Sum: %i + %i = %i', data.a, data.b, data.a + data.b)
    #rate = rospy.Rate(5)
    #rospy.loginfo(sum)
    rate = rospy.Rate(1)  
    
    # Publish the sum
    pub.publish(sum)
    rate.sleep()


def listener():
    # Initialize the node
    rospy.init_node('My_listener', anonymous=True)
    # Subscribe to the topic
    rospy.Subscriber("two_ints", TwoInts, callback)
    # Keep the node running
    rospy.spin()

if __name__=="__main__":
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

