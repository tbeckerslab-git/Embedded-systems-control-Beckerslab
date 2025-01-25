#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int16

def talker():
    pub = rospy.Publisher('/set_torque', Int16, queue_size=10)
    rospy.init_node('example_actuator', anonymous=True)
    rate = rospy.Rate(15) # 10hz
    while not rospy.is_shutdown():
        for i in range(300):
            pub.publish(i)
            # pub.publish(100)
            rate.sleep()
        for i in range(300, 0, -1):
            pub.publish(i)
            # pub.publish(0)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass