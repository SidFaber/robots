#! /usr/bin/python3

import RPi.GPIO as GPIO
import time
pinTrigger = 17
pinEcho = 18

#  set GPIO modes
GPIO.setmode (GPIO.BCM)
GPIO.setwarnings (False)
GPIO.setup (pinTrigger, GPIO.OUT)
GPIO.setup (pinEcho, GPIO.IN)

try:
    while True:
        GPIO.output (pinTrigger, False)
        time.sleep (0.5)

        #  Send a 10us pulse
        print("send pulse")
        GPIO.output (pinTrigger, True)
        time.sleep (0.00001)
        GPIO.output (pinTrigger, False)

        #  Wait for echo to go high, then low
        StartTime = time.time()
        while GPIO.input (pinEcho) == 0:
            StartTime = time.time()
        StopTime = time.time()
        while GPIO.input (pinEcho) == 1:
            StopTime = time.time()
            if StopTime - StartTime >= 0.04:
                StopTime = StartTime
                print ("Too Close")
                break
        
        ElapsedTime = StopTime - StartTime
        #  sound travels 34326 cm per second
        #distanceCM = ElapsedTime * 34326 / 2
        #  sound travels 13504 inches per second
        distanceIPS = ElapsedTime * 13504 / 2
        print ("%.1f in" % distanceIPS)

        time.sleep (0.5)

except KeyboardInterrupt:
    GPIO.cleanup()

