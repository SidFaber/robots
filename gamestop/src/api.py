#
#  Start with 
# export FLASK_APP=api.py
# flask run --host=0.0.0.0
#
from flask import Flask
from flask import render_template
from arm import Arm
import rclpy
from threading import Thread
import sys

class ArmThread (Thread):
    """A worker thread to let the arm spin and update work in progress"""
    def __init__ (self, arm):
        super().__init__()
        self.arm = arm

    def run (self):
        print ("spinning")
        rclpy.spin (self.arm)

try:
    rclpy.init (args=sys.argv)
except RuntimeError as err:
    # initialized for a second time
    print ("Caught {0}".format(err))
    pass

arm = Arm ("armapi")
armthread = ArmThread (arm)
armthread.start()

app = Flask(__name__)


@app.route('/api')
def hello_world():
    # spin for up to 1/20s to wait for the arm to update
    #for i in range(0, 20):
    #    rclpy.spin_once(arm, timeout_sec=0.01)
    return arm.json_status()

