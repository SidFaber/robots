#! /bin/python3
#
#  This code derived largely from the ROS 2 tutorials,
#  https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber/
#
import os
import rclpy
from rclpy.node import Node
from random import random
from std_msgs.msg import String
import sys

class MinimalPublisher(Node):

    def __init__(self, msg = ""):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'microk8s', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.msg = msg

    def timer_callback(self):
        msg = String()
        msg.data = '%s: %d' % (self.msg, self.i)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(msg = ""):
    rclpy.init(args=None)
    minimal_publisher = MinimalPublisher(msg)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    msg = ""
    if len (sys.argv) == 2:
        msg += sys.argv[1] + ":"
    msg += os.environ['HOSTNAME']
    msg += ":" + str(int(random()*1000))
    main(msg)

