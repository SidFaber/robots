#! /usr/bin/env python3
import time
from motor import Motor

#ROS 2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Wheelie:
    '''A robot with two PWM driven motors'''

    def __init__ (self, name, pinRightFwd, pinRightRev, pinLeftFwd, pinLeftRev, frequency = 20):
        self._frequency = frequency
        self.rightWheel = Motor(pinRightFwd, pinRightRev, frequency)
        self.leftWheel = Motor(pinLeftFwd, pinLeftRev, frequency)

    def stop (self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def forward (self, speed = 100):
        self.rightWheel.forward (speed)
        self.leftWheel.forward (speed)

    def reverse (self, speed = 100):
        self.rightWheel.reverse (speed)
        self.leftWheel.reverse (speed)

    def left (self, speed = 100):
        self.rightWheel.reverse (speed)
        self.leftWheel.forward (speed)

    def right (self, speed = 100):
        self.rightWheel.forward (speed)
        self.leftWheel.reverse (speed)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'move',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.wheelie = Wheelie(pinRightFwd=10, pinRightRev=9, pinLeftFwd=8, pinLeftRev=7)

    def listener_callback(self, msg):
        command = msg.data
        if command == 'forward':
            print('Moving forward')
            self.wheelie.goForward()
        elif command == 'reverse':
            print('Moving backward')
            self.wheelie.goBackward()
        elif command == 'left':
            print('Turning left')
            self.wheelie.goBackward()
        elif command == 'right':
            print('Turning right')
            self.wheelie.goRight()
        elif command == 'stop':
            print('Stopping')
            self.wheelie.stop()
        else:
            print('Unknown command, stopping instead')
            self.wheelie.stop()

def main(args=None):

    w = Wheelie("poolcity", pinRightFwd=10, pinRightRev=9, pinLeftFwd=8, pinLeftRev=7)
    w.forward()
    time.sleep (1)
    w.reverse()
    time.sleep(1)
    w.stop()
    time.sleep(1)
    w.left()
    time.sleep(1)
    w.right()
    time.sleep(1)
    w.stop()

    # #  initialize the wheelie node
    # rclpy.init(args=args)
    # minimal_subscriber = MinimalSubscriber()

    # #  wait for incoming commands
    # rclpy.spin(minimal_subscriber)

    # #  Interrupt detected, shut down
    # minimal_subscriber.wheelie.stop()
    # GPIO.cleanup()
    # minimal_subscriber.destroy_node()
    # rclpy.shutdown()



if __name__ == '__main__':
    main()

