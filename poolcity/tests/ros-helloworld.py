#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

GPIO.setmode (GPIO.BCM)
GPIO.setwarnings (False)

class Motor:
    def __init__ (self, pinFwd, pinBack, frequency=20, maxSpeed=100):
        #  Configure GPIO
        GPIO.setup (pinFwd,  GPIO.OUT)
        GPIO.setup (pinBack, GPIO.OUT)

        #  get a handle to PWM
        self._frequency = frequency
        self._maxSpeed = maxSpeed
        self._pwmFwd  = GPIO.PWM (pinFwd,  frequency)
        self._pwmBack = GPIO.PWM (pinBack, frequency)
        self.stop()

    def forwards (self, speed):
        self.move (speed)

    def backwards (self, speed):
       self.move (-speed)

    def stop (self):
        self.move (0)

    def move (self, speed):
        #  set limits
        if speed > self._maxSpeed:
            speed = self._maxSpeed
        if speed < -self._maxSpeed:
            speed = -self._maxSpeed

        #  turn on the motors
        if speed < 0:
            self._pwmFwd.start(0)
            self._pwmBack.start(-speed)
        else:
            self._pwmFwd.start(speed)
            self._pwmBack.start(0)

class Wheelie:
    def __init__ (self):
        self.rightWheel = Motor (10, 9)
        self.leftWheel = Motor (8, 7)

    def stop (self):
        self.leftWheel.stop()
        self.rightWheel.stop()

    def goForward (self, speed = 100):
        self.rightWheel.forwards (speed)
        self.leftWheel.forwards (speed)

    def goBackward (self, speed = 100):
        self.rightWheel.backwards (speed)
        self.leftWheel.backwards (speed)

    def goLeft (self, speed = 100):
        self.rightWheel.backwards (speed)
        self.leftWheel.forwards (speed)

    def goRight (self, speed = 100):
        self.rightWheel.forwards (speed)
        self.leftWheel.backwards (speed)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'move',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.wheelie = Wheelie()

    def listener_callback(self, msg):
        command = msg.data
        if command == 'forward':
            print('Moving forward')
            self.wheelie.goForward()
        elif command == 'backward':
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
    #  initialize the wheelie node
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()

    #  wait for incoming commands
    rclpy.spin(minimal_subscriber)

    #  Interrupt detected, shut down
    minimal_subscriber.wheelie.stop()
    GPIO.cleanup()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

