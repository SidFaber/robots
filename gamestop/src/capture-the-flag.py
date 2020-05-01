import rclpy
import sys
import time
from arm import Arm

def main (argv):
    rclpy.init (args=argv)
    arm = Arm()

    # set up the approach
    arm.move_to (+0.2015, +0.1000, +0.0850)
    arm.open()
    
    #capture
    arm.move_to (+0.2670, +0.1400, +0.0850, accelerate=0.2)
    arm.close()
    time.sleep (0.5)

    #raise
    arm.move_to (arm.x, arm.y, 0.1000)

    #celebrate
    arm.move_to (+0.1943, +0.0000, +0.3000)
    arm.move_joints (joint4=-0.2000, path_time=0.4)
    arm.move_joints (joint4=+1.0500, path_time=0.4)
    arm.move_joints (joint4=-0.2000, path_time=0.4)
    arm.move_joints (joint4=+1.0500, path_time=0.4)
    arm.move_joints (joint4=-0.2000, path_time=0.4)
    arm.move_joints (joint4=+1.0500, path_time=0.4)

    arm.move_joints (joint1=-0.9127, joint2=+0.9802, joint3=-1.3116, joint4=-0.2654, path_time=1.5)
    time.sleep(2.0)
    arm.move_joints (joint1=-0.3022, joint2=-0.7409, joint3=+0.8820, joint4=-0.1488, path_time=1.5)

    #return to the other side
    arm.move_to (+0.2670, -0.1400, +0.2000)
    arm.find_surface()
    # account for some slack in the arm
    arm.move_to (arm.x, arm.y, arm.z+0.0350, accelerate=0.2)
    arm.move_gripper(0.0000)
    arm.move_to (+0.2015, -0.1000, arm.z, accelerate=0.2)

    #go to neutral
    arm.move_to (+0.2200, +0.0000, +0.1000)

if __name__ == '__main__':
    main(sys.argv[1:])
