#! /usr/bin/env python3
import time
from range_sensor import UltrasonicDistanceSensor

#ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class Sensors (Node):
    '''List of sensors for a robot
    
    Attributes
    ----------
    distance : float
        Last distance measurement received from the sensor

    Methods
    -------
    stop()
        Stop all sensor readings
    start()
        Start sensor readings

    '''

    def __init__(self, name,
        pinTrigger, pinEcho, frequency=10):

        """
        Parameters
        ----------
        name: str
            The node name that will be used for this robot; defaults
            to "wheelie"
        pinTrigger : int
            The RaspPi GPIO pin that goes high for 10us to trigger the
            ultrasonic sensor
        pinEcho : int
            The RaspPi GPIO pin that goes high for the duration of when
            the sound is sent until it's received back
        frequency : int
            The frequency in Hz for how often measurements are requested
        """

        super().__init__(name)

        self._frequency = frequency
        self._distance_sensor = UltrasonicDistanceSensor(pinTrigger, pinEcho)
        self._distance_publisher = self.create_publisher(Range, "range", 5)
        self._range_msg = Range()
        self._range_msg.radiation_type = Range.ULTRASOUND
        self._range_msg.field_of_view = self._distance_sensor.angle
        self._range_msg.min_range = self._distance_sensor.min_range
        self._range_msg.max_range = self._distance_sensor.max_range

        self.start()

    def _distance_callback (self):
        self.distance = self._distance_sensor.measure()
        self._range_msg.range = self.distance
        self._distance_publisher.publish(self._range_msg)

    def stop (self):
        self.destroy_timer(self._distance_timer)

    def start (self):
        self._distance_timer = self.create_timer (1.0 / self._frequency, self._distance_callback)


def main(args=None):

    rclpy.init (args=args)
    s = Sensors("range", pinTrigger = 17, pinEcho = 18)
    print ("Spinning.")
    rclpy.spin (s)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

