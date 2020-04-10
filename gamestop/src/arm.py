import rclpy
from rclpy.node import Node

'''
ros2 topic list -t
...
/open_manipulator_x/joint_states [sensor_msgs/msg/JointState]
...
python3
help ('modules sensor_msgs.msgs')
import inspect
inspect.getmembers(sensor_msgs.msg, inspect.isclass)
'''
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from geometry_msgs.msg import Pose, Point, Quaternion

from threading import Timer

class Arm(Node):
    position = None
    x = 0
    y = 0
    z = 0

    qw = 0
    qx = 0
    qy = 0
    qz = 0    

    moving = ""
    actuator = ""

    timer = None

    def __init__(self):
        super().__init__('arm')

        self.position = {
            "joint1" : 0,
            "joint2" : 0,
            "joint3" : 0,
            "joint4" : 0,
            "gripper" : 0
            }

        # Topic: /open_manipulator_x/joint_states
        # Type: sensor_msgs/msg/JointState
        # 100Hz
        self.jointstate_sub = self.create_subscription(
            JointState,
            "/open_manipulator_x/joint_states",
            self.jointstates_callback,
            0)

        # Topic: /open_manipulator_x/kinematics_pose
        # Type: open_manipulator_msgs/msg/KinematicsPose
        # 100Hz
        self.kinematicspose_sub = self.create_subscription(
            KinematicsPose,
            "/open_manipulator_x/kinematics_pose",
            self.kinematicspose_callback,
            0)
            
        # Topic: /open_manipulator_x/states
        # Type: open_manipulator_msgs/msg/OpenManipulatorState
        # 100Hz
        self.state_sub = self.create_subscription(
            OpenManipulatorState,
            "/open_manipulator_x/states",
            self.state_callback,
            0)
            

            
    def jointstates_callback(self, msg):
        """ Record the current joint state"""
        for joint in range (0, len(msg.name)):
            self.position[msg.name[joint]] = msg.position[joint]

    def kinematicspose_callback(self, msg):
        """ Record the current kinematic pose"""
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.qw = msg.pose.orientation.w
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
    
    def state_callback (self, msg):
        """Record the current robot state"""
        self.moving = msg.open_manipulator_moving_state
        self.actuator = msg.open_manipulator_actuator_state

    def print_status (self):
        print (chr(27) + "[2J")
        print ("""
Joints: [1] %f  [2] %f  [3] %f  [4] %f
Gripper: %f

Position: (%f, %f, %f)
Quaternion: (%f, %f, %f, %f)

Moving: %s
Actuator: %s
"""
         % (
             self.position["joint1"],
             self.position["joint2"],
             self.position["joint3"],
             self.position["joint4"],
             self.position["gripper"], 
             self.x, self.y, self.z, 
             self.qw, self.qx, self.qy, self.qz, 
             self.moving, self.actuator
             ))
        Timer (1, self.print_status).start()

def main(args=None):
    rclpy.init(args=args)

    arm = Arm()
    arm.print_status()
    rclpy.spin(arm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
