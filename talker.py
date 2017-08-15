#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('rsbconverter', String, queue_size=10)
    rospy.init_node('rsbconverter', anonymous=True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        data = "some data %s" % rospy.get_time()
        rospy.loginfo("Sending " + data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
