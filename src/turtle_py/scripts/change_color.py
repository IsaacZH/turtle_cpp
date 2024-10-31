#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
import random
import time

def main():
    rospy.init_node('background')
    clear_bg = rospy.ServiceProxy('/clear', Empty)
    random.seed()

    while not rospy.is_shutdown():
        rospy.set_param('/turtle1/background_r', random.randint(0, 255))
        rospy.set_param('/turtle1/background_b', random.randint(0, 255))
        rospy.set_param('/turtle1/background_g', random.randint(0, 255))

        clear_bg()
        time.sleep(1)

if __name__ == '__main__':
    main()