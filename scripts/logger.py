#!/usr/bin/env python3

import rospy
import time
from supervisor.msg import DataLogger
from assessment import Visualiser



def custom_publisher(time_start, true_n, true_velocity, predicted_n, predicted_velocity, key_event=0.0, key_event_time=0.0):
    start_time = time.time()
    rospy.init_node('DataLogger', anonymous=True)
    pub = rospy.Publisher('DataLogger', DataLogger, queue_size=10)
    data_logger = DataLogger()
    data_logger.time = start_time
    data_logger.true_n = true_n
    data_logger.true_velocity = true_velocity
    data_logger.predicted_n = predicted_n
    data_logger.predicted_velocity = predicted_velocity
    data_logger.key_event = key_event
    data_logger.key_event_time = key_event_time

    pub.publish(data_logger)


def callback(msg):
    rospy.loginfo(f'time: {msg.time_start}\n true_n: {msg.true_n}\n true_velocity: {msg.true_velocity}\n predicted_n: {msg.predicted_n}\n predicted_velocty: {msg.predicted_velocity} key_event: {msg.key_event}\n key_event_time{msg.key_event_time}')

def subscriber_ff():
    rospy.Subscriber('DataLogger', DataLogger, callback = callback)

if __name__ == '__main__':
    rospy.init_node('DataLogger_Node')
    vis = Visualiser()

    rate = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            custom_publisher(vis.cb_time, vis.load_factor_list[-1], vis.velocity_list[-1], vis.load_factor_prediction_list[-1], vis.velocity_prediction_list[-1])
            subscriber_ff()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
