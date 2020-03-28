#! /usr/bin/python3

import RPi.GPIO as GPIO
import time
pinLightSensor = 25

#  set GPIO
GPIO.setmode (GPIO.BCM)
GPIO.setwarnings (False)
GPIO.setup (pinLightSensor, GPIO.IN)

try:
    while True:
        #  sensor low == no light
        print (GPIO.input (pinLightSensor))
        time.sleep (0.2)

except KeyboardInterrupt:
    GPIO.cleanup()

