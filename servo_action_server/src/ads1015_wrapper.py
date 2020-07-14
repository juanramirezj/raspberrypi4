#!/usr/bin/env python

import rospy
from ads1015_driver import ADS1015

class ads1015Wrapper:
    def __init__(self):
        self.ads = ADS1015()


if __name__ == "__main__":
    rospy.init_node("ads_driver")
    adsWrapper = ads1015Wrapper()
    rospy.loginfo("Prueba ok...")
    rospy.spin()

