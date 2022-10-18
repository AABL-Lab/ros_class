#!/usr/bin/env python
# license removed for brevity
# @author james.staley625703@tufts.edu
import rospy
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from realbot.msg import Wander

'''
Main loop drives roomba around randomly until the bump sensor is detected then turns
'''

global ACTIVE, TURN, TURN_COUNT
ACTIVE = False
TURN = False
LINEAR_VEL_X = 0.2
ANGULAR_VEL_Z = 1.0

def active_cb(msg):
    global ACTIVE
    print("Active cb: ", msg)
    ACTIVE = msg.data

def bumper_cb(msg):
    global TURN, TURN_COUNT
    print("Bumped! ", msg)
    ''' 
    For now lets just turn a new way whenever we get bumped
    QUESTION FOR CLASS: 
        How could we use the information in this message to make a better decision?
    '''
    TURN = True
    TURN_COUNT = 15 # 1.5 seconds at 10 hz

global TMP_LIN_VEL, TMP_ANG_VEL, TMP_COUNTDOWN_S
TMP_COUNTDOWN_S = 0
TMP_LIN_VEL = 0
TMP_ANG_VEL = 0
def wander_cb(msg):
    global TMP_LIN_VEL, TMP_ANG_VEL, TMP_COUNTDOWN_S 
    TMP_LIN_VEL = msg.linear_velocity
    TMP_ANG_VEL = msg.angular_velocity
    TMP_COUNTDOWN_S = msg.run_time_seconds
    print("Setting temporary velocities to {:1.2f}, {:1.2f} for {:1.2f}s".format(TMP_LIN_VEL, TMP_ANG_VEL, TMP_COUNTDOWN_S))


def main():
    global TURN, TURN_COUNT, ACTIVE
    global TMP_LIN_VEL, TMP_ANG_VEL, TMP_COUNTDOWN_S 

    rospy.init_node('roomba', anonymous=True)
    hz = 10
    rate = rospy.Rate(hz) # 10hz
    
    bump_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_cb)
    active_sub = rospy.Subscriber("/set_active", Bool, active_cb)
    wander_sub = rospy.Subscriber("/set_wander", Wander, wander_cb)

    vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    while not rospy.is_shutdown():
        # MAIN CONTROL LOOP
        cmd = Twist()
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0

        temporary_velocities_active = False
        if TMP_COUNTDOWN_S > 0:
            TMP_COUNTDOWN_S = TMP_COUNTDOWN_S - 1. / hz
            temporary_velocities_active = True
            print("Temporary velocities active for ", TMP_COUNTDOWN_S)

        if not ACTIVE:
            print("inactive")
        elif TURN:
            if TURN_COUNT > 0:
                cmd.angular.z = TMP_ANG_VEL if temporary_velocities_active else ANGULAR_VEL_Z
                TURN_COUNT -= 1
            else:
                TURN = False
        else:
            cmd.linear.x = TMP_LIN_VEL if temporary_velocities_active else LINEAR_VEL_X

        vel_pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass