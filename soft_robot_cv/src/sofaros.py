# coding: utf8
import Sofa.Core
import rospy 
from std_msgs.msg import Float32MultiArray

class RosSender(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # to reduce the latency in TCP, we can disable Nagle's algo with tcp_nodelay=False in ROS1
        # (https://en.wikipedia.org/wiki/Nagle%27s_algorithm)
        # Todo: find the equivalent in ROS2
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosSender"

        # Args
        self.node = args[0]
        rosname = args[1]
        self.datafield = args[2]
        msgtype = args[3]
        self.sendingcb = args[4]

        # Create or connect to the topic rosname as a publisher
        # self.pub = self.node.Publisher(rosname, msgtype, queue_size=10)
        self.pub = rospy.Publisher(rosname, msgtype, queue_size=10)

    def onAnimateEndEvent(self, event):
        # print(self.datafield)
        print("ANIMATE END")
        data = self.sendingcb(self.datafield)
        # rate = rospy.Rate(2) # 10hz
        # rate.sleep()
        self.pub.publish(data)


class RosReceiver(Sofa.Core.Controller):

    def callback(self, data):
        print("SOFAROS callback1")
        self.data = data.data
        # self.onAnimateBeginEvent
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosReceiver"
        
        self.node = args[0]
        self.rosname = args[1]
        self.datafield = args[2]
        self.msgtype = args[3]
        self.recvcb = args[4]

        # Create or connect to the topic rosname as a subscription
        # self.sub = self.node.Subscriber(rosname, msgtype, self.callback)
        # print("HERE for message type init")
        # print(self.msgtype)
        # self.sub = rospy.Subscriber(self.rosname, self.msgtype, self.callback)

        self.data = None
        self.subscriber = rospy.Subscriber(self.rosname, self.msgtype, self.callback)
        # rospy.spin()
    

    def onAnimateBeginEvent(self, event):
        # rospy.spin_once(self.node, timeout_sec=0.001)  # Something must be hidden in this spin_once(), without it the callback() is never called
        # new_data = rospy.wait_for_message(self.rosname, self.msgtype, timeout=0.5) #for fps, timeout=0.004
        print("first wait")
        print(self.data)
        # print("Second wait")
        # new_data = rospy.wait_for_message(self.rosname, self.msgtype, timeout=0.2)
        # print("New data from animate")
        # print(new_data)
        # self.data = new_data.data

        if self.data is not None:
            print("CALLING RECVCB")
            self.recvcb(self.data, self.datafield)
            self.data = None


def init(nodeName="Sofa"):
    # rospy.init()
    rospy.init_node(nodeName, anonymous=False) #Make anonymous true later now/Careful because init_node doesn't actually return anything
    # node.get_logger().info('Created node')
    # return node
