#! /usr/bin/python3

import RPi.GPIO as GPIO
import time
import math

pinTrigger = 17
pinEcho = 18


class UltrasonicDistanceSensor:
    '''An HC-SR04 Ultrasonic Range Sensor

    Methods
    -------
    measure()
        Return the distance in meters seen by the sensor

    '''

    def __init__ (self, pinTrigger, pinEcho):
        # Configure GPIO
        GPIO.setmode (GPIO.BCM)
        GPIO.setwarnings (False)
        GPIO.setup (pinTrigger, GPIO.OUT)
        GPIO.setup (pinEcho, GPIO.IN)

        self._pinTrigger = pinTrigger
        self._pinEcho = pinEcho

        # assumes dry air at 20C
        self.speed_of_sound = 343.0

        # these come from the sensor datasheet and are used in creating
        # a ROS Range message.
        # https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
        self.min_range = 0.030
        self.max_range = 4.000
        self.angle = 15.0 * math.pi / 180.0  #degrees to radians

        # pre-compute some static values
        self._min_travel_time = 2.0 * self.min_range / self.speed_of_sound
        self._max_travel_time = 2.0 * self.max_range / self.speed_of_sound

    def measure (self):
        '''Take a measurement
        
        Send out a low pulse to clear the sensor. Send a high pulse for 10us
        and wait for trigger to go high. Measure how long it takes for trigger
        to go low again, and compute distance based on that time.
        '''

        #  Send a 10us pulse
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
                break
        elapsed_time = StopTime - StartTime

        return elapsed_time * self.speed_of_sound / 2

