import rclpy
import sys
import time
from arm import Arm

def main (argv):
    rclpy.init (args=argv)
    arm = Arm()
   
    #collapse together
    arm.move_joints (-0.0000, -0.1104, +1.5000, -1.3700, path_time=4.0)
    arm.move_joints (joint1=-1.5000, path_time=1.0)
    arm.move_joints (joint1=+1.5000, path_time=2.0)
    arm.move_joints (joint1=-1.5000, path_time=2.0)
    arm.move_joints (joint1=+1.5000, path_time=2.0)
    arm.move_joints (joint1=-1.5000, path_time=2.0)
    arm.move_joints (joint1=+1.5000, path_time=2.0)
    arm.move_joints (joint1=-1.5000, path_time=2.0)
    arm.move_joints (joint1=+1.5000, path_time=2.0)
    arm.move_joints (joint1=+0.0000, path_time=2.0)
    time.sleep (2.0)

    # return to neutral
    arm.move_joints (-0.1227, -1.0440, +0.6213, +0.4341, path_time=4.0)
    arm.move_to (+0.1360, +0.0000, +0.2324, accelerate = 0.5)

    time.sleep (3.0)

    # dizzy
    arm.move_joints (joint4=+0.7000, path_time=1.0)
    arm.circle (radius=0.010, path_time=2.0)
    arm.circle (radius=0.020, path_time=4.0)
    arm.circle (radius=0.010, path_time=2.0)

    # return to neutral
    arm.move_to (+0.1360, +0.0000, +0.2324, accelerate = 0.3)

if __name__ == '__main__':
    main(sys.argv[1:])
