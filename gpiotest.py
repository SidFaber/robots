#! /usr/bin/python3

import RPi.GPIO as GPIO
import time

#  set GPIO modes
GPIO.setmode (GPIO.BCM)
GPIO.setwarnings (False)

#  set GPIO pin mode
GPIO.setup (7, GPIO.OUT)
GPIO.setup (8, GPIO.OUT)
GPIO.setup (9, GPIO.OUT)
GPIO.setup (10, GPIO.OUT)

#  turn motors off
GPIO.output(7,0)
GPIO.output(8,0)
GPIO.output(9,0)
GPIO.output(10,0)

#  turn right motor forward
GPIO.output(7,0)
GPIO.output(8,1)

#  turn left motor forward
GPIO.output(9,0)
GPIO.output(10,1)

time.sleep(1)

#  Reset GPIO and turn off motors
GPIO.cleanup()

