#!/usr/bin/env python3
# This script runs at the cloud/edge: sends the command messages to the agents (done)

import rospy
import socket
import pickle
from nav_msgs.msg import Odometry


class SendData(socket.socket):

    def __init__(self, uri, port_num) -> None:
        self.uri = uri
        self.port_num = port_num
        self.addr_info = socket.getaddrinfo(uri, port_num, proto=socket.IPPROTO_UDP)
        super().__init__(
            self.addr_info[0][0], self.addr_info[0][1], self.addr_info[0][2]
        )
        rospy.loginfo(
            "Socket created uri: {sock_uri}, port num: {port_num}".format(
                sock_uri=uri, port_num=port_num
            )
        )

    def send_data(self, message):
        self.sendto(message, (self.uri, self.port_num))


def stack_data(data, socket):
    message = pickle.dumps(data)
    socket.send_data(message)


if __name__ == "__main__":
    try:
        capturer_node = rospy.init_node("udp_client_odom", anonymous=True)
        # public ip of edge vm
        socket = SendData("10.161.68.100", 30101)
        # odometry topic
        topic = "/shafter4/odometry/imu"
        subscriber = rospy.Subscriber(topic, Odometry, stack_data, callback_args=socket)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
