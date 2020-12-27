#! /usr/bin/env python3

#Raspberry Pi
import RPi.GPIO as GPIO

class Motor:
    '''A PWM DC motor driven by two pins on a Raspberry Pi.'''

    def __init__ (self, pinFwd, pinRev, frequency=20, maxSpeed=100):
        #  Configure GPIO
        GPIO.setmode (GPIO.BCM)
        GPIO.setwarnings (False)
        GPIO.setup (pinFwd,  GPIO.OUT)
        GPIO.setup (pinRev, GPIO.OUT)
        
        #  get a handle to PWM
        self._frequency = frequency
        self._maxSpeed = maxSpeed
        self._pwmFwd  = GPIO.PWM (pinFwd, frequency)
        self._pwmRev = GPIO.PWM (pinRev, frequency)
        self.stop()

    def forward (self, speed = None):
        '''Spin the motor'''
        if speed == None:
            speed = self._maxSpeed
        self.run (speed)

    def reverse (self, speed = None):
        '''Spin the motor in revers'''
        if speed == None:
            speed = self._maxSpeed
        self.run (-1 * speed)

    def stop (self):
        '''Stop the motor'''
        self.run (0)

    def run (self, speed=None):
        '''Start the motor. If no speed is given, starts moving
        forward at the max speed.'''
        
        #  set limits
        if speed == None:
            speed = self._maxSpeed
        speed = min (self._maxSpeed, speed)
        speed = max (-self._maxSpeed, speed)

        #  turn on the motors
        if speed < 0:
            self._pwmFwd.start(0)
            self._pwmRev.start(-1.0 * speed)
        else:
            self._pwmFwd.start(speed)
            self._pwmRev.start(0)
