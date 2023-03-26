#!/usr/bin/env python3

import rospy
import time
from offboard_py.msg import FourFloats  # Import your custom message type
from flight_envelope_assessment import Visualiser


def custom_publisher(msg1, msg2, msg3, msg4):
    start_time = time.time()
    rospy.init_node('logger', anonymous=True)
    pub = rospy.Publisher('load_factor', FourFloats, queue_size=10)
    four_floats = FourFloats()
    four_floats.value1 = msg1
    four_floats.value2 = msg2
    four_floats.value3 = msg3
    four_floats.value4 = msg4
    pub.publish(four_floats)


def callback(msg):
    rospy.loginfo("Received data: a = %f, b = %f, c = %f, d = %f" % (msg.a, msg.b, msg.c, msg.d))

def subscriber_ff():
    rospy.init_node('four_floats_subscriber', anonymous=True)
    rospy.Subscriber('four_floats_topic', FourFloats, callback)




if __name__ == '__main__':
    rospy.init_node('logging')
    vis = Visualiser()

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            custom_publisher(vis.time_list[-1], vis.load_factor_list[-1], vis.load_factor_prediction_list[-1], vis.vertical_acceleration_list[-1])
            subscriber_ff()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
