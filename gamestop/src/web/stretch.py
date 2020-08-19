import rclpy
import sys
import time
from arm import Arm

def main (argv):
    rclpy.init (args=argv)
    arm = Arm()
   
    #stretch up
    arm.move_joints (-0.000, +0.000, -1.6580, -0.0000, path_time=4.0)
    time.sleep (2.0)
    arm.move_joints (-0.1227, -1.0440, +0.6213, +0.4341, path_time=4.0)
    time.sleep (2.0)

    #stretch forward
    arm.move_joints (+0.0000, -1.5900 +0.2800, +1.5000, path_time=4.0)
    arm.move_joints (+0.0000, +1.4650, -1.6337, +0.2378, path_time=6.0)
    time.sleep (2.0)
    arm.move_joints (+0.0000, -1.5900, +0.2800, +1.5000, path_time=4.0)
    time.sleep (2.0)

    #stretch left
    arm.move_joints (-1.5447, -1.5900, +0.2800, +1.5000, path_time=4.0)
    arm.move_joints (-1.5447, +1.4650, -1.6337, +0.2378, path_time=6.0)
    time.sleep (2.0)
    arm.move_joints (-1.5447, -1.5900, +0.2800, +1.5000, path_time=4.0)
    time.sleep (2.0)

    #stretch right
    arm.move_joints (+1.5447, -1.5900, +0.2800, +1.5000, path_time=4.0)
    arm.move_joints (+1.5447, +1.4650, -1.6337, +0.2378, path_time=6.0)
    time.sleep (2.0)
    arm.move_joints (+1.5447, -1.5900, +0.2800, +1.5000, path_time=4.0)
    time.sleep (2.0)


    # return to neutral
    arm.move_joints (-0.1227, -1.0440, +0.6213, +0.4341, path_time=4.0)
    arm.move_to (+0.1360, +0.0000, +0.2324, accelerate = 0.5)

if __name__ == '__main__':
    main(sys.argv[1:])
