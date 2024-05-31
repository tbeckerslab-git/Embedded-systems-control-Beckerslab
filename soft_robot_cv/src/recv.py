#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import sys

pub = rospy.Publisher('/animation/receiver/position', Float32MultiArray, queue_size=10)


def callback(data):
    print("Publishing to animation")
    print(data)
    global pub
    # rate = rospy.Rate(0.25) # 10hz
    # rate.sleep()
    pub.publish(data)

def talker():
        node = rospy.init_node('listener', anonymous=False)
        # pub = rospy.Publisher('/animation/receiver/position', Float32MultiArray, queue_size=10)
        sub = rospy.Subscriber('/simulation/sender/position', Float32MultiArray, callback)
        
        rospy.spin()

        # first_msg = Float32MultiArray()
        # first_msg.data = [0.0, 0.0, 0.0]
        # pub.publish(first_msg)

        # rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     new_msg = rospy.wait_for_message('/simulation/sender/position', Float32MultiArray)
        #     # print("IN WHILE")
        #     # callback(new_msg)
        #     rate.sleep()
           

        
        # while not rospy.is_shutdown():
        #    hello_str = "hello world %s" % rospy.get_time()
        #    rospy.loginfo(hello_str)
        #    pub.publish(hello_str)
        #    rate.sleep()

    
   
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass