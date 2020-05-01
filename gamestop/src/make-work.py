import random
import rclpy
import sys
import time
from arm import Arm

def dizzy (arm):
   
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
    arm.circle (radius=0.020, revolution=3.5, path_time=7.0)
    arm.move_joints (joint4=+0.4587, path_time=2.0)

    # return to neutral
    arm.move_to (+0.1360, +0.0000, +0.2324, accelerate = 0.3)

def stretch (arm):
   
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

def peek (arm):
   
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


def random_call (arm):

    routine = random.randrange(1,5)
    if routine == 1:
        print ("peek")
        peek (arm)
    elif routine == 2:
        print ("dizzy")
        dizzy(arm)
    elif routine == 3:
        print ("stretch")
        stretch(arm)
    else:
        pass


def main(argv):
    rclpy.init(args=argv)
    arm = Arm()

    while True:
        print ("home")
        arm.move_to (+0.1370, +0.0000, +0.2315)
        print ("sleep")
        time.sleep (random.randrange (5, 30) + random.randrange (5, 30))
        random_call (arm)


if __name__ == '__main__':
    main(sys.argv[1:])
