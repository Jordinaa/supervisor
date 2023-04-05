#!/usr/bin/env python3

import rospy
import mavros
from mavros import param




if __name__ == "__main__":

    rospy.init_node("pti_node")
    rate = rospy.Rate(20)

    mavros.set_namespace()

    while not rospy.is_shutdown():
        
        pti_get = param.param_get("FTI_ENABLE")
        pti_get_amp = param.param_get("FTI_FS_AMP_BEGIN")
        print(pti_get)
        print(f'amp {pti_get_amp}')

        pti_set = param.param_set("FTI_ENABLE", 1)
        print(pti_set)
        rate.sleep()
