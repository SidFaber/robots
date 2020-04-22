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
import time

class ArmThread (Thread):
    """A worker thread to let the arm spin and update work in progress"""
    def __init__ (self, arm):
        super().__init__()
        self.arm = arm
        self.spinning = True


    def run (self):
        print ("spinning")
        while True:
            if self.spinning:
                rclpy.spin_once (self.arm)
            else:
                time.sleep(0.5)


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


@app.route('/')
def status():
    # spin for up to 1/20s to wait for the arm to update
    #for i in range(0, 20):
    #    rclpy.spin_once(arm, timeout_sec=0.01)
    return render_template ('status.html')

@app.route('/api')
def api():
    # spin for up to 1/20s to wait for the arm to update
    #for i in range(0, 20):
    #    rclpy.spin_once(arm, timeout_sec=0.01)
    return arm.json_status()

@app.route('/home')
def home():
    armthread.spinning = False
    time.sleep (0.1)

    arm.move_to (+0.2000, +0.0000, +0.2400, accelerate=2.0)
    arm.move_joints (joint4=+0.7000, path_time=1.0)
    time.sleep (2.0)
    arm.move_to (+0.2800, +0.0000, +0.2400, accelerate = 0.3)
    time.sleep (2.0)
    arm.open()
    arm.close()
    arm.open()
    arm.close()

    time.sleep(3.0)
    arm.move_to (+0.1796, +0.0000, +0.1500, accelerate=0.3)
    time.sleep(3.0)
    arm.open()
    arm.close()
    time.sleep(3.0)
    # return to neutral
    arm.move_to (+0.1360, +0.0000, +0.2324, accelerate = 0.5)
    armthread.spinning = True

    return ""

@app.route('/up')
def up():
    armthread.spinning = False
    time.sleep (0.1)

    arm.move_joints (-0.000, +0.000, -1.6580, -0.0000, path_time=4.0)
    time.sleep (2.0)
    arm.move_joints (-0.1227, -1.0440, +0.6213, +0.4341, path_time=4.0)

    armthread.spinning = True
    return ""

@app.route('/front')
def front():
    armthread.spinning = False
    time.sleep (0.1)

    arm.move_joints (+0.0000, -1.5900 +0.2800, +1.5000, path_time=4.0)
    arm.move_joints (+0.0000, +1.4650, -1.6337, +0.2378, path_time=6.0)
    time.sleep (2.0)
    arm.move_joints (+0.0000, -1.5900, +0.2800, +1.5000, path_time=4.0)

    armthread.spinning = True
    return ""
