********************
Calling ROS Services
********************


Overview
========

Services within the Robot Operating System (ROS) ROS services control robot behavior.
There's plenty of documentation on how to `create and use services<https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/>`.
This complements other tutorials by specfically focusing on getting started without code.
The service interface continues to evolve as ROS 2 matures, so be sure to work with the right version.

What you'll learn
-----------------
In this tutorial we'll cover how to discover and call ROS services.

What you'll need
----------------
* ROS 2 installed on your computer
* A robot (real or simulated) running ROS 2 to use for testing

The computer you use may be the same computer running the robot, or you can use another computer on the same network as the robot.

Applicability
-------------
This tutorial applies to ROS Dashing and the OpenManipulatorX robot arm.


Discover Services
=================
Launch your robot according to the manufacturer's instructions.
.. code-block:: bash

  cd robotis_ws
  source install/setup.bash
  ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

Use the ``ros2 service list`` command to list the services provided by your robot:
.. code-block:: bash

  ubuntu@ros2:~/robotis_ws$ ros2 service list
  /launch_ros/describe_parameters
  /launch_ros/get_parameter_types
  /launch_ros/get_parameters
  /launch_ros/list_parameters
  /launch_ros/set_parameters
  /launch_ros/set_parameters_atomically
  /open_manipulator_x/goal_drawing_trajectory
  /open_manipulator_x/goal_joint_space_path
  ...
This is a long list.  However, ROS 2 implements parameters as services within each node, so there are six services attached to each node by default.  If we filter out those services, we get a much smaller list and can group them by function (spacing added for readability):  
.. code-block:: bash

  ubuntu@ros2:~/robotis_ws$ ros2 service list|grep -v parameter
  /open_manipulator_x/goal_drawing_trajectory

  /open_manipulator_x/goal_joint_space_path
  /open_manipulator_x/goal_joint_space_path_from_present
  /open_manipulator_x/goal_joint_space_path_to_kinematics_orientation
  /open_manipulator_x/goal_joint_space_path_to_kinematics_pose
  /open_manipulator_x/goal_joint_space_path_to_kinematics_position

  /open_manipulator_x/goal_task_space_path
  /open_manipulator_x/goal_task_space_path_from_present
  /open_manipulator_x/goal_task_space_path_from_present_orientation_only
  /open_manipulator_x/goal_task_space_path_from_present_position_only
  /open_manipulator_x/goal_task_space_path_orientation_only
  /open_manipulator_x/goal_task_space_path_position_only

  /open_manipulator_x/goal_tool_control

  /open_manipulator_x/set_actuator_state

Combined with the `documentation for the robot<http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros2_controller_package/#service-server-list>`, we get a brief understanding of these services.  We'll initially target the set_actuator_state service to lock and unlock the servos on the arm.

Use ros2 service list -t to get the service type definition information from the service:
.. code-block:: bash

  ubuntu@ros2:~/robotis_ws$ ros2 service list -t | grep -v parameter
  ...
  /open_manipulator_x/set_actuator_state [open_manipulator_msgs/srv/SetActuatorState]

Note that many services can have the same interface type, but each service has only a single interface type defined.  The set_actuator_state service has an interface type “open_manipulator_msgs/srv/SetActuatorState”.


Discover the Service Interface
==============================

In order to interact with a service, messages must be sent and received according to the message type.  The type is described using the “ros2 srv” command.

.. topic:: “ros2 service” vs “ros2 srv”

  The Service verb is used to interact with the current actively running space.

  The srv verb is used to read service configuration information.  Although services may be configured, they may not be running depending on what you launch.

  ros2 service list shows all the services currently active in your ROS domain.  ros2 srv list shows all the services recognized and configured for your client to use.  A service may be installed but not currently in use, and a service may be in use by another node on another computer but not recognizable or configured properly on your workstation.

Use ros2 srv show to find both the request message type and the response message type:
.. code-block:: bash

  ubuntu@ros2:~/robotis_ws$ ros2 srv show open_manipulator_msgs/srv/SetActuatorState
  bool set_actuator_state
  ---
  bool is_planned

This output follows the interface definition language used for .srv service files.  For more see the `ROS2 documentation<https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/>`.   This service is pretty simple:  a boolean value of true or false.


Craft a Simple Message
======================
Use the set_actuator_state service to lock and unlock the arm.

Services are used through the ros2 service call command.  The command requires a service name and type (defined above) and the values required by the interface.  Values are all stored in `YAML format <https://yaml.org/>`.

In order to unlock the robot arm, we need to send the set_actuator_state service a YAML message containing a single variable set_actuator_state set to false.  From a bash prompt that looks like this:
.. code-block:: bash

  ubuntu@ros2:~/robotis_ws$ ros2 service call \ '/open_manipulator_x/set_actuator_state' \ 'open_manipulator_msgs/srv/SetActuatorState' \
    "{ set_actuator_state: false }"

  waiting for service to become available...
  requester: making request: open_manipulator_msgs.srv.SetActuatorState_Request(set_actuator_state=False)

  response:
  open_manipulator_msgs.srv.SetActuatorState_Response(is_planned=True)

The response is as expected:  a single variable named “is_planned” set to True.  The response is common for this robot arm and indicates that the robot is planning to carry out the requested task.

This command can be used in any script so long as the robot’s setup script has been called.


Craft a Complex Message
=======================
A slightly more complex message is to open and close the robot gripper.  This is done through the goal_tool_control service.  Using the ros2 service and ros2 srv commands, we can map out how to communicate with the service:

============= ============== ==============================
Service:      /open_manipulator_x/goal_tool_control
------------- ---------------------------------------------
Service Type: open_manipulator_msgs/srv/SetJointPosition
------------- ---------------------------------------------
Request Msg:  string         planning_group
------------- -------------- ------------------------------
|             JointPosition  joint_position
------------- -------------- ------------------------------
|             float64        path_time
============= ============== ==============================
	
Request YAML format:
::
  {
  planning_group: <string>,
  	path_time: <float>,
	  joint_position: {...}
  }

The request message is more complex.  planning_group and path_time are built-in types, but the variable joint_position is a message of type JointPosition.

Message types are also defined in the ROS 2 interface documentation, and consist of either built-in types or other message types.  In order to find the definition for the JointPosition message, use ros2 msg show:
.. code-block:bash

  ubuntu@ros2:~/robotis_ws$ ros2 msg show open_manipulator_msgs/msg/JointPosition
  string[]   joint_name
  float64[]  position
  float64    max_accelerations_scaling_factor
  float64    max_velocity_scaling_factor

Joint_name and position are array arguments which represent each joint to be controlled.  This lets us complete the YAML formatted request message:
::

  {
    planning_group: “string”,
    path_time: ##,
    joint_position: {
      joint_name: [<string1>, <string2>, ...],
      position: [<float1>, <float2>, ...],
      max_accelerations_scaling_factor: <float>,
      max_velocity_scaling_factor: <float>
    }
  }

After consulting the robot's documentation to understand the fields, we have enough information to send this service a message.  For this robot arm the planning_group, path_time and scaling_factor variables are related to MoveIt! motion planning and can be ignored.  Joint names are "gripper" and "joint1" through "joint4" (the arm axes).  Each joint has defined limits; the "gripper" joint is variable between -0.01 (closed) and 0.01 (open).  All measurements are in meters, and all angles are in radians.

The service call to open the joint is as follows:
.. code-block:bash

  ros2 service call \
      '/open_manipulator_x/goal_tool_control' \
      'open_manipulator_msgs/srv/SetJointPosition' \
  '{
    planning_group: “nnn”,
    path_time: 1,
    joint_position: {
      joint_name: ["gripper"],
      position: [0.01],
      max_accelerations_scaling_factor: 1.0,
      max_velocity_scaling_factor: 1.0
    }
  }'

Or, removing the unnecessary variables and simplify:
.. code-block:bash

  ros2 service call \
    '/open_manipulator_x/goal_tool_control' \
    'open_manipulator_msgs/srv/SetJointPosition' \
    '{ joint_position: { joint_name: ["gripper"], position: [0.01] } }'



Move to a Point in Space
========================
Explore the goal_task_space service the robot end effector (the gripper) to a specific (x, y, z) position in space using the SetKinematicsPose interface.
.. code-block:bash

  ubuntu@ros2:~/robotis_ws$ ros2 srv show open_manipulator_msgs/srv/SetKinematicsPose
  string planning_group
  string end_effector_name
  KinematicsPose kinematics_pose
  float64 path_time
  ---
  bool is_planned

  ubuntu@ros2:~/robotis_ws$ ros2 msg show open_manipulator_msgs/msg/KinematicsPose
  geometry_msgs/Pose  pose
  float64    max_accelerations_scaling_factor
  float64    max_velocity_scaling_factor
  float64    tolerance

  ubuntu@ros2:~/robotis_ws$ ros2 msg show open_manipulator_msgs/msg/KinematicsPose
  geometry_msgs/Pose  pose
  float64    max_accelerations_scaling_factor
  float64    max_velocity_scaling_factor
  float64    tolerance

  ubuntu@ros2:~/robotis_ws$ ros2 msg show geometry_msgs/Pose
  # A representation of pose in free space, composed of postion and orientation.

  Point position
  Quaternion orientation

  ubuntu@ros2:~/robotis_ws$ ros2 msg show geometry_msgs/Point
  # This contains the position of a point in free space
  float64 x
  float64 y
  float64 z

  ubuntu@ros2:~/robotis_ws$ ros2 msg show geometry_msgs/Quaternion
  # This represents an orientation in free space in quaternion form.

  float64 x
  float64 y
  float64 z
  float64 w

The following service call will move the robot arm to position (x=0, y=0, z=0):
.. code-block:bash

  ros2 service call \
    '/open_manipulator_x/goal_task_space_path' \
    'open_manipulator_msgs/srv/SetKinematicsPose' \
    '{
      planning_group: "abc", 
      end_effector_name: "gripper", 
      kinematics_pose: { 
        pose: {
          position: {
            x: 0.0,
            y: 0.0,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0
          },
        },
        max_accelerations_scaling_factor: 1.0,
        max_velocity_scaling_factor: 1.0,
        tolerance: 0.005
        },
      path_time: 3.0
      }'

Again removing the unnecessary arguments to simplify:
.. code-block:bash

  ros2 service call \
    '/open_manipulator_x/goal_task_space_path' \
    'open_manipulator_msgs/srv/SetKinematicsPose' \
    '{end_effector_name: "gripper", path_time: 3.0,
      kinematics_pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0 }}}}'


Draw a Circle
=============

Use /open_manipulator_x/goal_drawing_trajectory, interface type [open_manipulator_msgs/srv/SetDrawingTrajectory]
.. code-block:bash

  ros2 srv show open_manipulator_msgs/srv/SetDrawingTrajectory
  string end_effector_name
  string drawing_trajectory_name
  float64[] param
  float64 path_time
  ---
  bool is_planned

The trajectory name "circle" requires three parameters:  the radius of the circle in meters, the number of revolutions (1.0 is a full 360 degrees) and the start angle of the circle in radians.  The following call draws a circle of radius 3cm two times:
.. code-block:bash

  ros2 service call \
    '/open_manipulator_x/goal_drawing_trajectory' \
    'open_manipulator_msgs/srv/SetDrawingTrajectory' \
    '{
      end_effector_name: "gripper", 
      drawing_trajectory_name: "circle",
      path_time: 5.0,
      param: [0.03, 2.0, 0 ]
    }'


Conclusion!
===========
Congratulations! You're now able to control your robot to run some straight forward tasks without having to compile and run code. Your next step might be to create a simple shell script to run commands.  Here's a simple example:
.. code-block: bash

  #! /bin/bash
  usage() {
      echo "Usage: $0 verb

  Verbs:
    circle   Draw a circle
    close    Close the gripper
    help     This message
    home     Go to home position
    lock     Locks the arm servos
    neutral  Go to neutral position
    open     Open the gripper
    unlock   Unlocks the arm servos
  "
  }

  # source the environment if it hasn't already been
  if [ -z "ROS_VERSION" ]
  then
      source ~/ros2/robotis_ws/install/setup.bash
  fi

  if [ -z "$1" ]
  then
      usage
      exit 0
  fi

  case "$1" in
      "circle")
          ros2 service call \
              '/open_manipulator_x/goal_drawing_trajectory' \
              'open_manipulator_msgs/srv/SetDrawingTrajectory' \
              '{ end_effector_name: "gripper", 
                  drawing_trajectory_name: "circle",
                  path_time: 5.0,
                  param: [0.03, 1.0, 0 ]
                }'
      ;;
      
      "close" )
          ros2 service call \
              '/open_manipulator_x/goal_tool_control' \
              'open_manipulator_msgs/srv/SetJointPosition' \
              '{ joint_position: { joint_name: ["gripper"], position: [-0.01] } }'
      ;;
      
      "help" )
          usage
      ;;
      
      "home" )
          ros2 service call \
              '/open_manipulator_x/goal_task_space_path' \
              'open_manipulator_msgs/srv/SetKinematicsPose' \
              '{end_effector_name: "gripper", path_time: 3.0,
              kinematics_pose: {pose: {position: {x: 0.1356, y: 0.0000, z: 0.2363}}}}'
      ;;
      
      "lock" )
          ros2 service call \
              '/open_manipulator_x/set_actuator_state' \
              'open_manipulator_msgs/srv/SetActuatorState' \
              "{ set_actuator_state: true }"
      ;;
      "neutral" )
          ros2 service call \
              '/open_manipulator_x/goal_task_space_path' \
              'open_manipulator_msgs/srv/SetKinematicsPose' \
              '{end_effector_name: "gripper", path_time: 3.0,
              kinematics_pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0 }}}}'
      
      ;;
      "open" )
          ros2 service call \
              '/open_manipulator_x/goal_tool_control' \
              'open_manipulator_msgs/srv/SetJointPosition' \
              '{ joint_position: { joint_name: ["gripper"], position: [0.01] } }'
      ;;
      "unlock" )
          ros2 service call \
              '/open_manipulator_x/set_actuator_state' \
              'open_manipulator_msgs/srv/SetActuatorState' \
              "{ set_actuator_state: false }"
      ;;

      * )
          echo "ERROR: '$1' not recognized.\n"
          usage
  esac

