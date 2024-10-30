import rospy
import threading
from sensor_msgs.msg import PointCloud2


class Pub_Node(threading.Thread):
    
    def __init__(self, rate = 100, topic_name = "/test_topic", node_name = "test_name") -> None:
        threading.Thread.__init__(self)
        self._rate = rate
        self.topic_name = topic_name
        self.empty_message = PointCloud2()
        self.message = self.empty_message
        self.pub = rospy.Publisher(self.topic_name, PointCloud2, queue_size=2**8)
        if not rospy.core.is_initialized():
            self.udp_server_node = rospy.init_node('udp_server_' + node_name, anonymous=True)
    
    def run(self):
        n_rate = rospy.Rate(self._rate)

        while True and not rospy.is_shutdown():
            self.pub.publish(self.message)
            if hasattr(self.message, 'header'):
                self.empty_message.header = self.message.header
            self.message = self.empty_message
            n_rate.sleep()

    def create_msg(self, data):
        self.message = data

if __name__ == '__main__':
    try:
        pub_node = Pub_Node(rate=30, topic_name="/shafter4/ouster/points", node_name="test_node")
        pub_node.start()    
    except rospy.ROSInterruptException:
        print("rospy interrupted")
