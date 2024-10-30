#!/usr/bin/env python3
# This script runs at the cloud/edge: receives the odometry messages from the agents (done)

import rospy
import threading
import socket
import pickle
import sys
import ros_node as ros_node
from sensor_msgs.msg import PointCloud2


class SocketServer(threading.Thread):
    def __init__(
        self, uri, port_num, ros_node, timeout=None, large_message=False
    ) -> None:
        threading.Thread.__init__(self)
        self.uri = uri
        self.port_num = port_num
        self.addr_info = socket.getaddrinfo(uri, port_num, proto=socket.IPPROTO_UDP)
        self.socket = socket.socket(
            self.addr_info[0][0], self.addr_info[0][1], self.addr_info[0][2]
        )
        self.should_run = True
        if timeout != None:
            self.socket.settimeout(timeout)
        self.max_length = 65540
        self.ros_node = ros_node
        self.large_message = large_message

    def bind_socket(self):
        self.socket.bind((self.uri, self.port_num))
        print(self.addr_info)
        try:
            while self.should_run:
                data, addr = self.socket.recvfrom(self.max_length)
                if self.large_message == False:
                    self.publish_small_messages(data)
                else:
                    self.process_large_messages(data)

        except KeyboardInterrupt:
            print("interrupted!")
            my_socket.should_run = False
            raise Exception("Close Socket")

    def publish_small_messages(self, data):
        print("received data")
        header_data = pickle.loads(data)
        self.ros_node.create_msg(header_data)

    def process_large_messages(self, data):
        if len(data) < 100:
            frame_info = pickle.loads(data)
            if frame_info:
                nums_of_packs = frame_info["packs"]
                for i in range(nums_of_packs):
                    data, address = self.socket.recvfrom(self.max_length)
                    if i == 0:
                        buffer = data
                    else:
                        buffer += data
                try:
                    data_variable = pickle.loads(buffer)
                except Exception as e:
                    print("Corrupted Data")
                    return 0
                print("Received full frame.. + " + str(self.addr_info))
                self.ros_node.create_msg(data_variable)

    def run(self):
        print("Run socket thread...")
        print(self.addr_info)
        self.bind_socket()


if __name__ == "__main__":
    try:
        pub_node = ros_node.Pub_Node(
            rate=30, topic_name="/shafter4/ouster/points", node_name="udp_server_points"
        )
        pub_node.start()
        my_socket = SocketServer("192.168.10.176", 30104, pub_node, large_message=True)
        my_socket.daemon = True
        my_socket.start()
    except Exception as e:
        my_socket.should_run = False
        print("in exception")
        print(e)
