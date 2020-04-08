# Call ROS 2 Services - OpenManipulator X

## Overview

Duration: 3:00

Services within the Robot Operating System (ROS) ROS are often used to control your robot.
After getting your robot built and software installed, the next step is to make it do something!
Before a deep dive into writing software, let's create a few simple service calls to do some basic actions with your robot.

> **ROS or ROS 2?**
>
> [ROS 2](https://index.ros.org/doc/ros2/) is a complete overhaul of the [ROS](https://www.ros.org/), the de facto global standard for robotics R&D.
> ROS 2 builds upon the rich history of ROS with many improved features to meet the needs of commercial robots.
> Although it is possible to combine componets of both ROS and ROS 2 in a single robot, the versions are generally not compatible.
> 
> This tutorial is written exclusively for ROS 2.


### What you'll learn

In this tutorial we'll cover how to discover and call ROS 2 services on an 
[OpenMANIPULATOR-X](https://www.robotis.us/openmanipulator-x/)
robot arm.

Using a different robot?
No problem!
The commands are the same, but you'll get different results depending on the services made available with your robot.
So long as you're running ROS 2, you should be able to use this tutorial!

### What you'll need

All you need to complete this terminal is an OpenMANIPULATOR-X robot arm assembled and set up and set up for ROS 2 according to the 
[Robotis instructions](http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros2_setup/#ros-setup).


### Before beginning

Before working through this tutorial, launch your robot according to the Robotis instructions:

```
  cd robotis_ws
  source install/setup.bash
  ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

You should see a successful launch similar to the following:

```
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/xxxxxxxx
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Execute OpenManipulator-X Controller!!
[INFO] [open_manipulator_x_controller-1]: process started with pid [24857]
```

The arm should now stay locked in position and will remain torqued so long as this command is running.
Leave this running and work through the rest of this tutorial in a new terminal window.


## Discover Services

Duration: 12:00

Now that the robot is running, let's take a look to see what services it publishes.

### Find Services

Use the `ros2 service list` command to list all the services provided by your robot.
Your output should look like the following:

```
/launch_ros/describe_parameters
/launch_ros/get_parameter_types
/launch_ros/get_parameters
/launch_ros/list_parameters
/launch_ros/set_parameters
/launch_ros/set_parameters_atomically
/open_manipulator_x/goal_drawing_trajectory
/open_manipulator_x/goal_joint_space_path
...
```

This is a long list.  However, ROS 2 implements parameters as services within each node, so there are six services attached to each node by default.
You can use grep to filter out those services with this command:

```
ros2 service list|grep -v parameter
```

This results in a much smaller list and can group them by function (spacing added for readability):  

```
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
```

Although the service names seem pretty self-explanatory, some 
[documentation for the robot](http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros2_controller_package/#service-server-list>)
may also help to better understand the purpose of these services.

In the next step we'll dig into the the `set_actuator_state` service to lock and unlock the servos on the arm.


## Discover the Service Interface

Duration: 7:00

Communications within ROS happens through messages.  
In order to control a service, we need to figure out how to craft a message it understands.

### Find the service message type

An easy way to find the message type for the `set_actuator_state` service is to use the `-t` option when listing services:

```
ros2 service list -t | grep state
```

This shows the type of message expected by the service:

```
/open_manipulator_x/set_actuator_state [open_manipulator_msgs/srv/SetActuatorState]
```

Many services can use the same message type, but each service accepts only a single type of message.
For our example, the set_actuator_state service has an interface type `open_manipulator_msgs/srv/SetActuatorState`.


### Explore the message interface

Armed with the type of message, we now need to how to craft a message of the right type.
This requires exploring the actual message structure:  the message interface specification.
You can find the details for a message interface using the `interface` verb as follows:

```
ros2 interface show open_manipulator_msgs/srv/SetActuatorState
```

This returns two different message structures separated by dashes.
The first structure is the message type **sent** to the service, while the second structure is the message type **returned** by the service:

```
bool set_actuator_state
---
bool is_planned
```

This interface is quite simple: send a boolean true/false value, and receive a boolean true/false message in response.

> :exclamation: **Does `ros2 interface` generate an error?**
>
> The `interface` argument was introduced with ROS Eloquent.
> If you're using ROS Dashing or an earlier verison of ROS, use the `ros2 srv` command instead.
> Both `ros2 interface` and `ros2 srv` are identical, but I think you'll agree that `interface` is much less confusing!

To better understand service interfaces, compare the output of the command `ros2 interface list --only-srvs` (`ros2 srv list` on Dashing) and `ros2 service list`.
The `service` command shows services that are *currently running* on your robot, while the `interface` command shows services that are *configured* for your robot.
Services may be configured but not actually started; for instance, your robot may not have a feature that comes with the code base, or a service may be dynamically started based on external conditions.



## Send a simple message to a service

Duration: 2:00

Now we have everything we need to send a message to the robot.
It's time ot use the `ros2 service call` command to call the service.
This command takes three arguments:
 - **service_name**: The name of the service
 - **service_type**: The type of the service
 - **values**: The message sent to the service

The message values need are sent in [YAML format](https://yaml.org/).
Since our message simply requires a single boolean value named `set_actuator_state` as shown by the message interface, encoding this YAML is fairly straight forward.

The final command looks like this:

```
ros2 service call \
  '/open_manipulator_x/set_actuator_state' \
  'open_manipulator_msgs/srv/SetActuatorState' \
  '{ set_actuator_state: false }'
```

You should see the following response:

```
waiting for service to become available...
requester: making request: open_manipulator_msgs.srv.SetActuatorState_Request(set_actuator_state=False)

response:
open_manipulator_msgs.srv.SetActuatorState_Response(is_planned=True)
```

The response matches our expectations: a single variable named “is_planned” set to True which means the robot plans to carry out our request.

Congratulations!
You've now sent your first service command to your robot.
Lock the arm again (hint: send the same message, but set the value to "true") and let's work with something a bit more complex.



## Craft a Complex Message

Duration: 5:00

Our next challenge will be to open and close the robot gripper.


### Find the service interface
The gripper can be accessed through the goal_tool_control service.
Using the `ros2 service` and `ros2 interface` commands we can map out how to communicate with this service:

A slightly more complex message is to open and close the robot gripper.  This is done through the goal_tool_control service.  Using the ros2 service and ros2 srv commands, we can map out how to communicate with the service:

 - Service: /open_manipulator_x/goal_tool_control
 - Service Type: open_manipulator_msgs/srv/SetJointPosition
 - Request Msg:
   - string: planning_group
   - JointPosition: joint_position
   - float64: path_time

Hmm.  The argumnets planning_group and path_time are 
[primitive field types](https://design.ros2.org/articles/interface_definition.html)
(string and float64), but the variable joint_position is a *message* of type JointPosition.
This is message type encapsulated within the service request.
In order to find out more about this JointPosition message type, use `ros2 interface list` to show the fully qualified message name:

```
Messages:
    action_msgs/msg/GoalInfo
    action_msgs/msg/GoalStatus
    ...
    open_manipulator_msgs/msg/JointPosition
    open_manipulator_msgs/msg/KinematicsPose
    open_manipulator_msgs/msg/OpenManipulatorState
    ...

```

Use the `msg show` command to show details about the `JointPosition` message:

```
ros2 interface show open_manipulator_msgs/msg/JointPosition
```

This command gives us details about the message structure:

```
  string[]   joint_name
  float64[]  position
  float64    max_accelerations_scaling_factor
  float64    max_velocity_scaling_factor
```

Note the brackets for the joint_name and joint_position arguments.
This means these are array arguments, so the YAML value sent to the service must send an array, not an individual value.

> :exclamation: **Get another error?**
>
> If you're using ROS Dashing or an earlier verison of ROS, use the `ros2 msg show` command instead of `ros2 interface show` and everything will work fine.

We've come a long way without referring to vendor documentation, but at some point you may find that you have to actually read the manual.
A quick look at the documentation for the robot arm lets us know the joint we're trying to move is called "gripper", and its position must be a value between -0.01 (fully closed) and 0.01 (fully open).

We now have enough information to create a well formed service call which opens the joint:

```
ros2 service call \
  '/open_manipulator_x/goal_tool_control' \
  'open_manipulator_msgs/srv/SetJointPosition' \
'{
  planning_group: "",
  path_time: 1,
  joint_position: {
    joint_name: ["gripper"],
    position: [0.01],
    max_accelerations_scaling_factor: 1.0,
    max_velocity_scaling_factor: 1.0
  }
}'
```

Another quick look at the documentation shows that we can take default values most of the parameters and remove them from the message.
Thsi results in a fairly simple command to open the gripper:

```
  ros2 service call \
    '/open_manipulator_x/goal_tool_control' \
    'open_manipulator_msgs/srv/SetJointPosition' \
    '{ joint_position: { joint_name: ["gripper"], position: [0.01] } }'
```

Can you create a service call to close the gripper?


## Move to a Point in Space

Duration: 5:00

We're on a roll.
Now let's make the arm move!

### Set a pose

In order to move the arm to a specific postion, we'll use the SetKinematicsPose interface.
Run through a few commands discussed earlier to get information similar to the following:

| Message Type | Field Type | Field Name |
|---|---|---|
| open_manipulator_msgs/srv/SetKinematicsPose | string | planning_group |
| | string | end_effector_name |
| | KinematicsPose | kinematics_pose |
| | float64 | path_time |
| open_manipulator_msgs/msg/KinematicsPose | geometry_msgs/Pose | pose |
| | float64 | max_accelerations_scaling_factor |
| | float64 | max_velocity_scaling_factor |
| | float64 | tolerance |
| geometry_msgs/msgs/Pose | Point | position |
| | Quaternion | orientation |
| | geometry_msgs/msgs/Point | float64 | x |
| | float64 | y |
| | float64 | z |
| geometry_msgs/msgs/Quaternion | float64 | x |
| | float64 | y |
| | float64 | z |
| | float64 | w |

Although this looks complex because message types are embedded within other message types, the process is identical to the example we just completed.
Fill in the YAML structure with the information gathered from exploring the message structure to move the robot arm to position (x=0, y=0, z=0):

```
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
```

Did your robot arm move?
Actually, for this robot moving to (0, 0, 0) may be unreasonable.
Instead try moving to (0.15, 0.00, 0.20) and see how that works!

ROS has standardized on the [International System of Units](https://en.wikipedia.org/wiki/International_System_of_Units) for all measurements.
Keep in mind that all distances are in meters and all angles are in radians.

Once again we can remove some unnecessary arguments (for now) to simplify:

```
  ros2 service call \
    '/open_manipulator_x/goal_task_space_path' \
    'open_manipulator_msgs/srv/SetKinematicsPose' \
    '{end_effector_name: "gripper", path_time: 3.0,
      kinematics_pose: {pose: {position: {x: 0.15, y: 0.0, z: 0.2 }}}}'
```

## Conclusion

Duration: 1:00

Congratulations!
You're now able to control your robot to run some straight forward tasks without having to compile and run code.

Continue exploring the wealth of documentation on how to 
[create and use services](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)
The service interface continues to evolve as ROS 2 matures, so be sure to check back often!

We'll leave you with a simple bash script to show how to pull this all together into a simple utility to manage your robot:

```
#! /bin/bash
usage() {
    echo "Usage: $0 verb
Verbs:
  close    Close the gripper
  help     This message
  home     Go to home position
  lock     Locks the arm servos
  open     Open the gripper
  unlock   Unlocks the arm servos
"
}

if [ -z "$1" ]
then
    usage
    exit 0
fi
case "$1" in
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
```
