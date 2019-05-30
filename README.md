# UR5_With_ROS_Moveit_Tutorial
A tutorial on setting up a UR CB arm to run with ROS and Moveit!

There are a few tricks to setting up a UR arm for use with ROS, after working through these I decided to document the process for other users. This tutorial is specifically how I set up a UR5 CB series arm with Moveit! although it should be a similar process for any other arm supported by the ur_modern_driver package. 

It is based on the instructions [here](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial) and modified to use the ur_modern_driver.

# Requirements
-To begin with you should have ROS kinetic installed by following http://wiki.ros.org/kinetic/Installation/Ubuntu 

-I tested this code with the UR polyscope version 3.9.1 but previous versions should also work. If you are updating the polyscope software you need to do each subversion in order (3.4.X, 3.5.X, 3.6.X and so on). You can get these files from the UR website, you will need a USB to copy them onto.

-You need a network connection to the arm, the simplest method is to set DHCP under the settings->network menu and read the IP address once it appears there. You can also run a direct ethernet connection using the 'Configure Your Hardware' instructions [here](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial). Your computer should ideally be on the same subnet as the arm. 

-Note the IP of the robot and check you can ping it using 
`ping IP_FROM_ABOVE`

# Installations
Follow only the installation portions of the instructions from https://github.com/ros-industrial/universal_robot and https://github.com/ros-industrial/ur_modern_driver/. If you also try to execute the demonstration commands the robot may not behave as you expect due to the issues described in the 'Known Problems' section.


-Download the Universal Robot repo from https://github.com/ros-industrial/universal_robot into your catkin workspace. Do not use the apt-get command but compile from source by following the 'Building from Source' instructions. **Important: Make sure you clone the kinetic devel branch using:**

`cd ~/catkin_ws/src
git clone -b kinetic-devel git@github.com:ros-industrial/universal_robot.git`

-Build your catkin workspace by running

`cd ~/catkin_ws
catkin_make clean
catkin_make install`

If you get any errors at this stage you will need to stop and fix them, these are typically due to missing dependencies which may be fixed by running

`rosdep update
rosdep install --rosdistro kinetic --ignore-src --from-paths src`

-Download the ur_modern_driver package kinect-devel branch into your catkin workspace using 

`cd ~/catkin_ws/src
git clone -b kinetic-devel git@github.com:ros-industrial/ur_modern_driver.git`

This was tested in June 2019, so if there are later breaking changes you may need to grab that version of the repository. 

then rebuild your catkin workspace with 

`cd ~/catkin_ws
catkin_make clean
catkin_make install`

You now have all of the required packages installed.

# Known Problems
There are three major issues with the default setup of the packages you installed:

1) The controllers are dependent on the speed setting on the UR tablet. At 100% speed the moveit motion commands and demonstration scripts (like the ur_driver test_move.py script) move extremely quickly. At lower speeds the controller gains are incorrect leading to over/undershoot.

2) Moveit! fails to create reasonable plans without reduced joint angle limits.

3) The ur_modern_driver by default uses a velocity controller. This is ideal for tasks such as visual servoing but most applications are better served by a position controller. So the default setting is changed to use this in my provided files.

# Using ros_control

There are several methods for controlling the robot using the ur_modern_driver package. I will cover how to use 2 of these. The first is through a position based controller running in ros_control. To launch this by default you need to change the ur5_ros_control.launch file slightly. (Or the appropriate launch file for your robot if using UR10 or UR3).

Run

`cd ~/catkin_ws/src/ur_modern_driver/launch
gedit ur5_ros_control.launch`

And change the following lines from

`<arg name="controllers" default="joint_state_controller force_torque_sensor_controller vel_based_pos_traj_controller"/>`

to

`<arg name="controllers" default="joint_state_controller force_torque_sensor_controller pos_based_pos_traj_controller"/>`

and change

`<arg name="stopped_controllers" default="pos_based_pos_traj_controller joint_group_vel_controller"/>`

to

`<arg name="stopped_controllers" default="vel_based_pos_traj_controller joint_group_vel_controller"/>`

This will now launch the position controller by default. You also need to add the position controller to the list of moveit! controllers. Replace the `controllers.yaml` file in `catkin_ws/src/universal_robot/ur5_moveit_config/config` with the one from this repository. If using UR10 / UR3 make sure the alter the correct controllers file. 

# Using the Low Bandwidth Controller

If you are operating over wifi or with high CPU loads on the ROS computer, you may need to use the low bandwidth controller. In order to do this...
