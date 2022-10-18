#!/usr/bin/env python

from __future__ import print_function

from realbot.srv import Undock, UndockResponse
from std_msgs.msg import Bool
import rospy
import time

def undock_behavior():
    rospy.wait_for_service("/undock")
    print("Found undock service.")

    try:
        u = rospy.ServiceProxy("/undock", Undock)
        resp1 = u("gene parmesan")
        return resp1.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    rospy.init_node('behavior')
    action_pub = rospy.Publisher("/set_active", Bool, queue_size=1)
    rate = rospy.Rate(10)

    # undock and wander for 30 seconds
    t0 = time.time()
    undock_behavior()

    action_pub.publish(True)
    while not rospy.is_shutdown() and time.time() - t0 < 30.0:
        print("running for ", time.time() - t0)
        rate.sleep()
    action_pub.publish(False)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
