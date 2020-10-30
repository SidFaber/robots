import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetKinematicsPose, SetJointPosition, SetDrawingTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import sys
import time
import json
from threading import Timer

#TODO: detect a lockout when one of the wait functions spins for too long.

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

class Joint (object):
    def __init__ (self, name):
        self.effort = 0
        self.name = name
        self.position = 0
        self.velocity = 0
        self.targetposition = 0

class Arm(Node):
    def __init__(self, node_name = "arm"):

        super().__init__(node_name)

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
        # published at 100Hz
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

        # Service: /open_manipulator_x/goal_tool_control
        # Interface: open_manipulator_msgs/srv/SetJointPosition
        self.move_gripper_service = self.create_client (
            SetJointPosition,
            "/open_manipulator_x/goal_tool_control")
        while not self.move_gripper_service.wait_for_service (timeout_sec = 1.0):
            self.get_logger().info("Waiting for goal_tool_control service")
        self.gripper_request = SetJointPosition.Request()
        self.gripper_future = None

        # Service: /open_manipulator_x/goal_joint_space_path
        # Interface: open_manipulator_msgs/srv/SetJointPosition
        self.move_joint_space_service = self.create_client (
            SetJointPosition,
            "/open_manipulator_x/goal_joint_space_path")
        while not self.move_joint_space_service.wait_for_service (timeout_sec = 1.0):
            self.get_logger().info("Waiting for goal_joint_space_path service")
        self.joint_request = SetJointPosition.Request()
        self.joint_future = None

        # Service: /open_manipulator_x/goal_drawing_trajectory
        # Interface: open_manipulator_msgs/srv/SetDrawingTrajectory
        self.move_trajectory_service = self.create_client (
            SetDrawingTrajectory,
            "/open_manipulator_x/goal_drawing_trajectory")
        while not self.move_trajectory_service.wait_for_service (timeout_sec = 1.0):
            self.get_logger().info("Waiting for goal_joint_space_path service")
        self.trajectory_request = SetDrawingTrajectory.Request()
        self.trajectory_future = None

               
    #######################################################################
    #  Listener callbacks
    #######################################################################

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


    #######################################################################
    #  X,Y position-related movement
    #######################################################################

    def wait_for_move_complete (self):
        """Blocks until movement is complete."""
        tolerance = 0.03
        while True:
            distance_to_go = math.sqrt (
                (self.x - self.targetx)**2 +
                (self.y - self.targety)**2 +
                (self.z - self.targetz)**2)
            if distance_to_go < tolerance:
                break
            rclpy.spin_once (self, timeout_sec = 0.1)
        self.stop_moving()


    def move_to (self, x, y, z, blocking = True, accelerate = 1.0):
        """Moves to a specific x, y, z coordinate."""

        accelerate = max(0.1, accelerate)
        accelerate = min(5.0, accelerate)
        self.targetx = x
        self.targety = y
        self.targetz = z

        distance_to_move = math.sqrt ((self.x - x)**2 + (self.y - y)**2 + (self.z - z)**2 )
        if distance_to_move < 0.03:
            return
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

    #######################################################################
    #  Joint-related movement
    #######################################################################

    def wait_for_joint_complete (self):
        """Blocks until movement is stopped."""
        
        # give the arm a chance to catch up if movement hasn't
        # started yet
        tolerance = 0.02
        spin = True
        spincount = 0
        while spin:
            spin = False
            spincount += 1
            if abs (self.joint1.targetposition - self.joint1.position) > tolerance:
                spin = True
            if abs (self.joint2.targetposition - self.joint2.position) > tolerance:
                spin = True
            if abs (self.joint3.targetposition - self.joint3.position) > tolerance:
                spin = True
            if abs (self.joint4.targetposition - self.joint4.position) > tolerance:
                spin = True
            if spincount > 1000:
                spin = False  #we've waited long enough by now.
            rclpy.spin_once (self, timeout_sec = 0.1)
        self.stop_moving()
            
    def move_joints (self, joint1 = None, joint2 = None, joint3 = None, joint4 = None, path_time = 5.0):
        """Moves to a specific joint configuration."""

        if joint1 == None:
            self.joint1.targetposition = self.joint1.position
        else:
            self.joint1.targetposition = joint1

        if joint2 == None:
            self.joint2.targetposition = self.joint2.position
        else:
            self.joint2.targetposition = joint2

        if joint3 == None:
            self.joint3.targetposition = self.joint3.position
        else:
            self.joint3.targetposition = joint3

        if joint4 == None:
            self.joint4.targetposition = self.joint4.position
        else:
            self.joint4.targetposition = joint4

        self.joint_request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        self.joint_request.joint_position.position = [
            self.joint1.targetposition, self.joint2.targetposition, 
            self.joint3.targetposition, self.joint4.targetposition]

        self.joint_request.path_time = path_time
        self.joint_future = self.move_joint_space_service.call_async (self.joint_request)
        rclpy.spin_until_future_complete(self, self.joint_future)
        self.wait_for_joint_complete()


    #######################################################################
    #  Trajectory movemement
    #######################################################################

    def wait_for_trajectory_complete (self):
        """Blocks until movement is stopped."""
        
        # start by looking for moving state to not be stopped
        spin = True
        spincount = 0
        while spin:
            spincount += 1
            if self.moving != "STOPPED":
                spin = False
            if spincount > 10:
                spin = False
            rclpy.spin_once (self, timeout_sec = 0.1)

        # should be moving by now, wait until stopped
        spin = True
        while spin:
            if self.moving == "STOPPED":
                spin = False
            if spincount > 100:
                spin = False
            rclpy.spin_once (self, timeout_sec = 0.1)


    def circle (self, radius=0.03, revolution=1.0, start_angle=0.0, path_time = 5.0):
        # this trajectory takes a radius, revolution, start_angular_position.
        self.trajectory_request.path_time = path_time
        self.trajectory_request.end_effector_name = "gripper"
        self.trajectory_request.drawing_trajectory_name = "circle"
        self.trajectory_request.param = [radius, revolution, start_angle]
        self.trajectory_future = self.move_trajectory_service.call_async (self.trajectory_request)
        rclpy.spin_until_future_complete(self, self.trajectory_future)
        self.wait_for_trajectory_complete()


    #######################################################################
    #  Gripper movement
    #######################################################################

    def move_gripper (self, pos):
        pos = max (-0.01, pos)
        pos = min (+0.01, pos)
        self.gripper_request.joint_position.joint_name = ["gripper"]
        self.gripper_request.joint_position.position = [float(pos)]
        self.gripper_future = self.move_gripper_service.call_async (self.gripper_request)
        rclpy.spin_until_future_complete(self, self.gripper_future)
        time.sleep(0.5)

    def open (self):
        """Open the gripper"""
        self.move_gripper (0.01)

    def close (self):
        """Open the gripper"""
        self.move_gripper (-0.01)


    #######################################################################
    #  effort-related movement
    #######################################################################

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


    #######################################################################
    #  Utilities
    #######################################################################

    def print_status (self):
        """Returns a string of the current state of the arm"""
        return (chr(27) + """[2J
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
        
    def json_status (self):
        """Returns a json object representing the current state of the arm"""

        #TODO: rewrite this in generic ROS message formats
        curr = {
            "position": {
                "x": round(self.x, 4),
                "y": round(self.y, 4),
                "z": round(self.z, 4)
            },
            "targetposition": {
                "x": round(self.targetx, 4),
                "y": round(self.targety, 4),
                "z": round(self.targetz, 4)
            },
            "quaternion": {
                "w": round(self.ow, 4),
                "x": round(self.ox, 4),
                "y": round(self.oy, 4),
                "z": round(self.oz, 4)
            },
            "joint1" : {
                "position": round(self.joint1.position, 4),
                "targetposition": round(self.joint1.targetposition, 4),
                "velocity": round(self.joint1.velocity, 4),
                "effort": round(self.joint1.effort, 4)
            },
            "joint2" : {
                "position": round(self.joint2.position, 4),
                "targetposition": round(self.joint2.targetposition, 4),
                "velocity": round(self.joint2.velocity, 4),
                "effort": round(self.joint2.effort, 4)
            },
            "joint3" : {
                "position": round(self.joint3.position, 4),
                "targetposition": round(self.joint3.targetposition, 4),
                "velocity": round(self.joint3.velocity, 4),
                "effort": round(self.joint3.effort, 4)
            },
            "joint4" : {
                "position": round(self.joint4.position, 4),
                "targetposition": round(self.joint4.targetposition, 4),
                "velocity": round(self.joint4.velocity, 4),
                "effort": round(self.joint4.effort, 4)
            },
            "gripper" : {
                "position": round(self.gripper.position, 4),
                "targetposition": round(self.gripper.targetposition, 4),
                "velocity": round(self.gripper.velocity, 4),
                "effort": round(self.gripper.effort, 4)
            },
            "moving": self.moving,
            "actuator": self.actuator
        }
        
        return json.dumps (curr)

    def reprint_status (self):
        self.print_status()
        Timer (1, self.reprint_status).start()

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


    def record_height (self, x, y):
        self.move_to (self.x, self.y, 0.2000, accelerate = 1.5)
        self.move_to (x, y, 0.2000, accelerate = 1.5)
        self.find_surface()
        height = self.z
        self.move_to (x, y, 0.2000, accelerate = 1.5)
        return height

    def stop_moving (self):
        """Make the current position the planned position."""

        self.targetx = self.x
        self.targety = self.y
        self.targetz = self.z

        self.joint1.targetposition = self.joint1.position
        self.joint2.targetposition = self.joint2.position
        self.joint3.targetposition = self.joint3.position
        self.joint4.targetposition = self.joint4.position
        self.gripper.targetposition = self.gripper.position

        # lock in the current joint position
        self.joint_request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        self.joint_request.joint_position.position = [
            self.joint1.targetposition, self.joint2.targetposition, 
            self.joint3.targetposition, self.joint4.targetposition]
        self.joint_request.path_time = 0.5
        self.joint_future = self.move_joint_space_service.call_async (self.joint_request)
        rclpy.spin_until_future_complete(self, self.joint_future)

        # self.move_request.path_time = 0.1
        # self.move_request.end_effector_name = "gripper"
        # self.move_request.kinematics_pose.pose.position.x = self.targetx
        # self.move_request.kinematics_pose.pose.position.y = self.targety
        # self.move_request.kinematics_pose.pose.position.z = self.targetz

        # self.move_future = self.move_task_space_service.call_async (self.move_request)
        # rclpy.spin_until_future_complete(self, self.move_future)


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
