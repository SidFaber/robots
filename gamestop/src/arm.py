import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import sys
import time

'''
ROS builds lots of stuff for you, even if it's not always documented
by the manufacturer.

Here's how to find python info to subscribe to an existing listener:

ros2 topic list -t
...
/open_manipulator_x/joint_states [sensor_msgs/msg/JointState]
...
python3
help ('modules sensor_msgs.msgs')
import inspect
inspect.getmembers(sensor_msgs.msg, inspect.isclass)

Here's how to find python info to use a service:
python3
help ('modules open_manipulator_msgs.srv')
import inspect
inspect.getmembers(open_manipulator_msgs.srv, inspect.isclass)

'''
from threading import Timer

class Joint (object):
    def __init__ (self, name):
        self.effort = 0
        self.name = name
        self.position = 0
        self.velocity = 0

class Arm(Node):
    def __init__(self):
        super().__init__('arm')

        self.x = 0
        self.y = 0
        self.z = 0

        self.targetx = 0
        self.targety = 0
        self.targetz = 0

        self.ow = 0
        self.ox = 0
        self.oy = 0
        self.oz = 0    

        self.moving = ""
        self.actuator = ""

        self.joint1 = Joint("joint1")
        self.joint2 = Joint("joint2")
        self.joint3 = Joint("joint3")
        self.joint4 = Joint("joint4")
        self.gripper = Joint("gripper")

        self.holding_pencil = False
        self.drawing = False

        self.velocity = 0.1  # 10cm/sec
        self.baseplate = 0.1000  # z location of the baseplate according to the pointer
        
        #===================================================
        # Listen on status nodes
        #
        # Topic: /open_manipulator_x/joint_states
        # Type: sensor_msgs/msg/JointState
        # pubilshed at 100Hz
        self.jointstate_sub = self.create_subscription(
            JointState,
            "/open_manipulator_x/joint_states",
            self.jointstates_callback,
            0)

        # Topic: /open_manipulator_x/kinematics_pose
        # Type: open_manipulator_msgs/msg/KinematicsPose
        # published at 100Hz
        self.kinematicspose_sub = self.create_subscription(
            KinematicsPose,
            "/open_manipulator_x/kinematics_pose",
            self.kinematicspose_callback,
            0)
            
        # Topic: /open_manipulator_x/states
        # Type: open_manipulator_msgs/msg/OpenManipulatorState
        # published at 100Hz
        self.state_sub = self.create_subscription(
            OpenManipulatorState,
            "/open_manipulator_x/states",
            self.state_callback,
            0)

        #======================================================
        # Subscribe to services
        #
        # Service: /open_manipulator_x/goal_task_space_path
        # Interface: open_manipulator_msgs/srv/SetKinematicsPose
        self.move_task_space_service = self.create_client (
            SetKinematicsPose,
            "/open_manipulator_x/goal_task_space_path")
        while not self.move_task_space_service.wait_for_service (timeout_sec = 1.0):
            self.get_logger().info("Waiting for goal_task_space_path service")
        self.move_request = SetKinematicsPose.Request()
        self.move_future = None

        # Service: /open_manipulator_x/goal_joint_space_path
        # Interface: open_manipulator_msgs/srv/SetJointPosition
        self.move_joint_space_service = self.create_client (
            SetJointPosition,
            "/open_manipulator_x/goal_tool_control")
        while not self.move_joint_space_service.wait_for_service (timeout_sec = 1.0):
            self.get_logger().info("Waiting for goal_joint_space_path service")
        self.joint_request = SetJointPosition.Request()
        self.joint_future = None

    def jointstates_callback(self, msg):
        """Record the current joint state"""

        #TODO: avoid the hard coded array position for each joint
        self.joint1.effort = msg.effort[0]
        self.joint1.position = msg.position[0]
        self.joint1.velocity = msg.velocity[0]

        self.joint2.effort = msg.effort[1]
        self.joint2.position = msg.position[1]
        self.joint2.velocity = msg.velocity[1]

        self.joint3.effort = msg.effort[2]
        self.joint3.position = msg.position[2]
        self.joint3.velocity = msg.velocity[2]

        self.joint4.effort = msg.effort[3]
        self.joint4.position = msg.position[3]
        self.joint4.velocity = msg.velocity[3]

        self.gripper.effort = msg.effort[0]
        self.gripper.position = msg.position[0]
        self.gripper.velocity = msg.velocity[0]


    def kinematicspose_callback(self, msg):
        """ Record the current kinematic pose"""
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

        self.ow = msg.pose.orientation.w
        self.ox = msg.pose.orientation.x
        self.oy = msg.pose.orientation.y
        self.oz = msg.pose.orientation.z
    
    def state_callback (self, msg):
        """Record the current robot state"""
        self.moving = msg.open_manipulator_moving_state
        self.actuator = msg.open_manipulator_actuator_state
        # identify when a requested move has started

    def status (self):
        """Returns a string of the current state of the arm"""
        return ("""
           ------------------------------------------------------------------
           | %10s | %10s | %10s | %10s | %10s |
-----------------------------------------------------------------------------
| Position | %+10.4f | %+10.4f | %+10.4f | %+10.4f | %+10.4f |
| velocity | %+10.4f | %+10.4f | %+10.4f | %+10.4f | %+10.4f |
| Effort   | %+10.4f | %+10.4f | %+10.4f | %+10.4f | %+10.4f |
-----------------------------------------------------------------------------

             -----------------------------------------------------
             |          W |          X |          Y |          Z |
------------------------------------------------------------------
| Position:  |            | %+10.4f | %+10.4f | %+10.4f |
| Quaternion | %+10.4f | %+10.4f | %+10.4f | %+10.4f |
------------------------------------------------------------------
Moving State:   %s
Actuator State: %s
"""
            % (
                self.joint1.name, self.joint2.name, self.joint3.name, self.joint4.name, self.gripper.name,
                self.joint1.position, self.joint2.position, self.joint3.position, self.joint4.position, self.gripper.position,
                self.joint1.velocity, self.joint2.velocity, self.joint3.velocity, self.joint4.velocity, self.gripper.velocity,
                self.joint1.effort, self.joint2.effort, self.joint3.effort, self.joint4.effort, self.gripper.effort,
                self.x, self.y, self.z, 
                self.ow, self.ox, self.oy, self.oz, 
                self.moving, self.actuator
                ))
        
    def print_status (self):
        print (chr(27) + "[2J" + self.status())

    def reprint_status (self):
        self.print_status()
        Timer (1, self.reprint_status).start()

    def stop_moving (self):
        """Make the current position the planned position."""

        self.targetx = self.x
        self.targety = self.y
        self.targetz = self.z

        self.move_request.path_time = 0.1
        self.move_request.end_effector_name = "gripper"
        self.move_request.kinematics_pose.pose.position.x = self.targetx
        self.move_request.kinematics_pose.pose.position.y = self.targety
        self.move_request.kinematics_pose.pose.position.z = self.targetz

        self.move_future = self.move_task_space_service.call_async (self.move_request)
        rclpy.spin_until_future_complete(self, self.move_future)


    def move_to (self, x, y, z, blocking = True, accelerate = 1.0):
        """Moves to a specific x, y, z coordinate."""

        self.targetx = x
        self.targety = y
        self.targetz = z

        distance_to_move = math.sqrt ((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2 )
        time_to_move = distance_to_move / self.velocity
        self.move_request.path_time = time_to_move / accelerate

        self.move_request.end_effector_name = "gripper"
        self.move_request.kinematics_pose.pose.position.x = self.targetx
        self.move_request.kinematics_pose.pose.position.y = self.targety
        self.move_request.kinematics_pose.pose.position.z = self.targetz

        self.move_future = self.move_task_space_service.call_async (self.move_request)
        rclpy.spin_until_future_complete(self, self.move_future)

        if blocking:
            self.wait_for_move_complete()

    def open (self):
        """Open the gripper"""

        self.joint_request.joint_position.joint_name = ["gripper"]
        self.joint_request.joint_position.position = [0.0000]
        self.joint_future = self.move_joint_space_service.call_async (self.joint_request)

        rclpy.spin_until_future_complete(self, self.joint_future)
        time.sleep(0.5)

    def close (self):
        """Open the gripper"""

        self.joint_request.joint_position.joint_name = ["gripper"]
        self.joint_request.joint_position.position = [-0.01]
        self.joint_future = self.move_joint_space_service.call_async (self.joint_request)

        rclpy.spin_until_future_complete(self, self.joint_future)
        time.sleep(0.5)

    def wait_for_move_complete (self):
        """Blocks until movement is complete."""

        while True:
            distance_to_go = math.sqrt (
                (self.x - self.targetx)**2 +
                (self.y - self.targety)**2 +
                (self.z - self.targetz)**2)
            if distance_to_go < 0.02:
                break
            rclpy.spin_once (self, timeout_sec = 0.1)
        self.stop_moving()

    def wait_for_move_torqued (self):
        """Blocks until done or joint 4 hits effort limit"""

        while True:
            if self.joint2.effort > 10:
                break
            rclpy.spin_once (self)  # wait for a callback to update joint effort
        self.stop_moving()

    def wait_for_move_untorqued (self):
        """Blocks until done or joint 4 hits zero effort"""

        while True:
            if self.joint2.effort <= 0:
                break
            rclpy.spin_once (self)  # wait for a callback to update joint effort
        self.stop_moving()

    def load (self):
        """Move to a neutral position, ask for a pencil and tension the gripper"""
        if self.holding_pencil:
            return
        
        #self.move_to (0.1500, 0.0000, 0.1000)
        self.move_to (0.2440, 0.0000, 0.1500)
        self.open()
        self.close()
        self.open()
        input ("Place the pointer and press enter:")
        self.close()
        self.baseplate = self.z

    def find_surface (self):
        """Move down until torqued, move up until torque is zero, then record the touch point"""

        self.targetx = self.x
        self.targety = self.y
        self.targetz = 0.0


        # move down
        self.move_request.path_time = 6.0
        self.move_request.end_effector_name = "gripper"
        self.move_request.kinematics_pose.pose.position.x = self.targetx
        self.move_request.kinematics_pose.pose.position.y = self.targety
        self.move_request.kinematics_pose.pose.position.z = self.targetz
        self.move_future = self.move_task_space_service.call_async (self.move_request)
        rclpy.spin_until_future_complete(self, self.move_future)
        self.wait_for_move_torqued()

        # back off
        self.targetx = self.x
        self.targety = self.y
        self.targetz = self.z + 0.02
        self.move_request.path_time = 2.0
        self.move_request.end_effector_name = "gripper"
        self.move_request.kinematics_pose.pose.position.x = self.targetx
        self.move_request.kinematics_pose.pose.position.y = self.targety
        self.move_request.kinematics_pose.pose.position.z = self.targetz
        self.move_future = self.move_task_space_service.call_async (self.move_request)
        rclpy.spin_until_future_complete(self, self.move_future)
        self.wait_for_move_untorqued()
        time.sleep (0.5)    # let the arm settle


    def record_height (self, x, y):
        self.move_to (self.x, self.y, 0.2000, accelerate = 1.5)
        self.move_to (x, y, 0.2000, accelerate = 1.5)
        self.find_surface()
        height = self.z
        self.move_to (x, y, 0.2000, accelerate = 1.5)
        return height


def main(argv):
    rclpy.init(args=argv)
    arm = Arm()

    points = []
    if len(argv) > 0:
        verb = argv[0]
    else:
        verb = "measure"

    if verb == "status":
        arm.reprint_status()
    elif verb == "load":
        arm.load()
    elif verb == "measure":
        arm.load()

        for xval in range (200, 260, 30):
            for yval in range (-110, 110, 50):
                height = arm.record_height(xval / 1000.0, yval / 1000.0)
                print ("[%10.4f %10.4f %10.4f]" % (arm.x, arm.y, height))
                points.append ((arm.x, arm.y, height))
                time.sleep(1.0)

    else:
        arm.load()

    print (points)

    rclpy.spin(arm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
