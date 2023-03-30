#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import FTI

def send_fti_injection():
    rospy.init_node('fti_injector', anonymous=True)
    fti_pub = rospy.Publisher('/mavros/fti/inject', FTI, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    # Create FTI injection message
    fti = FTI()
    fti.timestamp = rospy.Time.now()
    fti.fti_type = FTI.FTI_TYPE_ATTITUDE
    fti.param1 = 0.1 # roll angle in radians
    fti.param2 = 0.2 # pitch angle in radians
    fti.param3 = 0.3 # yaw angle in radians
    fti.param4 = 0.4 # thrust percentage (0-1)
    fti.param5 = 0.0 # not used
    fti.param6 = 0.0 # not used
    fti.param7 = 0.0 # not used

    while not rospy.is_shutdown():
        # Publish FTI injection message
        fti_pub.publish(fti)
        rospy.loginfo('Sent FTI injection')
        rate.sleep()

if __name__ == '__main__':
    try:
        send_fti_injection()
    except rospy.ROSInterruptException:
        pass
