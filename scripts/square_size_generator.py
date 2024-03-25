#!/usr/bin/env python
import random
import rospy
from test.msg import size


def square_size_generator():
    # Publish to "size" topic and set rate
    pub = rospy.Publisher('size', size , queue_size=10)
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(0.05) 


    # Generate the random values for square size
    while not rospy.is_shutdown():
        SS = size()
        SS.size = random.uniform(0.05,0.20)
        # Print the message
        rospy.loginfo("square size=%f",SS.size)

        # Publish to the topic
        pub.publish(SS)        
        rate.sleep()
if __name__ == '__main__':
    try:
        square_size_generator()
    except rospy.ROSInterruptException:
        pass
