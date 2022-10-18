#!/usr/bin/env python

from __future__ import print_function

from realbot.srv import Undock, UndockResponse
from geometry_msgs.msg import Twist
import rospy
import time

linear_msg = Twist()
linear_msg.angular.x = 0
linear_msg.angular.y = 0
linear_msg.angular.z = 0
linear_msg.linear.x = -0.2
linear_msg.linear.y = 0
linear_msg.linear.z = 0

angular_msg = Twist()
angular_msg.angular.x = 0
angular_msg.angular.y = 0
angular_msg.angular.z = 3.14/2.
angular_msg.linear.x = 0
angular_msg.linear.y = 0
angular_msg.linear.z = 0

def undock(req):
    print("Got undock request for ", req.robot_name)
    pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    # back up
    t0 = time.time()
    while (time.time() - t0 < 2.0):
        pub.publish(linear_msg)

    # turn around
    t0 = time.time()
    while (time.time() - t0 < 2.2):
        pub.publish(angular_msg)

    return UndockResponse(True)

def main():
    rospy.init_node('undock_server')
    s = rospy.Service('undock', Undock, undock)
    print("Ready to accept undock requests.")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
