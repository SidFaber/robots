import rclpy
import sys
import time
from arm import Arm

def main (argv):
    rclpy.init (args=argv)
    arm = Arm()
   
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

if __name__ == '__main__':
    main(sys.argv[1:])
