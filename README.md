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

A copy of both the ur_modern_driver and universal_robotics package has been included here, so that you don't have to worry about versioning. Each of these repositories retains its own licence and also has documentation in the respective folder, they are only included here for convenience. Note that we are using the kinetic-devel branches for both of these.

Run the following commands to install them, this assumes your catkin_ws folder has been created in the user home directory.

This command will clone the current repository, you can run it from anywhere **except** from within the catkin_ws folder.

`git clone git@github.com:jaspereb/UR5_With_ROS_Moveit_Tutorial.git`

`cd UR5_With_ROS_Moveit_Tutorial/`

`DL_DIR=$PWD`

Install the universal_robot package

`cp -r universal_robot/ ~/catkin_ws/src/`

`cd ~/catkin_ws`

`catkin_make clean`

`catkin_make`

`catkin_make install`

If you get any errors at this stage you will need to stop and fix them, these are typically due to missing dependencies which may be fixed by running

`rosdep update`

`rosdep install --rosdistro kinetic --ignore-src --from-paths src`

Install the ur_modern_driver package

`cd $DL_DIR`

`cp -r ur_modern_driver/ ~/catkin_ws/src/`

`cd ~/catkin_ws`

`catkin_make clean`

`catkin_make`

`catkin_make install`

You now have all of the required packages installed.

# Known Problems
There are three major issues with the default setup of the packages you installed:

1) The controllers are dependent on the speed setting on the UR tablet. At 100% speed the moveit motion commands and demonstration scripts (like the ur_driver test_move.py script) move extremely quickly. At lower speeds the controller gains are incorrect leading to over/undershoot.

You can get around this be using the low bandwidth controller as described below.

2) Moveit! fails to create reasonable plans without reduced joint angle limits. So you need to pass the joint limits argument to the ur5_moveit_config launch script. 

3) The ur_modern_driver by default uses a velocity controller. This is ideal for tasks such as visual servoing but most applications are better served by a position controller. So the default setting is changed to use this in my provided files.

# Arm Safety
**Important:** make sure to have the area around the arm completely clear of people and obstacles before enabling the motors. If you do not reduce the default speed the arm will move very quickly and could easily crash into the table or people. You should always be ready on the e-stop button when executing any move command, and be careful of any arm attachements (eg end effectors) which are not modelled in moveit! as these could be run into.

If you do reduce the default speed the ros_control approach will cause the arm to significantly overshoot waypoints, so if you have one close to an obstacle it may crash. 

# Using the Low Bandwidth Trajectory Follower

There are several methods for controlling the robot using the ur_modern_driver package. I will cover how to use 2 of these. The Low Bandwidth Trajectory Follower (LBTF) is the preferred method and should be sufficient unless you have high precision and feedback requirements. If you are operating over wifi or with high CPU loads on the ROS computer, you will need to use the LBTF.

To set this up you need to use the default `controllers.yaml` file from the universal_robot package. So if you updated this in order to use the ros_control method, you need to replace the `~/catkin_ws/src/universal_robot/ur5_moveit_config/config/controllers.yaml` file with the file from this repo in the `LBTF/controllers.yaml` location.

You also need to alter the `~/catkin_ws/src/ur_modern_driver/launch/ur5_bringup.launch` file within the ur_modern_driver package. There is an updated version provided, so copy this from `LBTF/ur5_bringup_LBTF.launch` into the location above.

I have changed the default `time_interval` parameter in this file to 0.004. The arm speed is the ratio of this to the `servoj_time` so this will run it at 50% speed. If you change this to 0.008 it will be back to 100% speed, but 50% is a good starting point the first time you run the driver. You should not have the same undershoot/overshoot problems that the ros_control method does when running at slower speeds.

## Launching with LBTF

To actually control the arm once you have made the above changes you need to establish a connection, enable the motors, launch the driver, launch the planner and finally launch Rviz. To do this:

1. Power on the arm, enable the DHCP setting and take note of the IP address once it appears

2. Check you can ping the arm  

3. Open an empty program, enable the arm motors and drive the arm to the home position

4. You should now be in the 'Move' menu on the UR tablet. If you are in the movement execution window (the one that has an 'Auto' and a 'Manual' button) then the driver will not work.

Run each of the following commands in a different terminal.

5. Run `roslaunch ur_modern_driver ur5_bringup_LBTF.launch robot_ip:=xxx.xxx.xxx.xxx` using the IP address you found before

6. Run `roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch` and wait for this to read `You can start planning now!`

7. Run `roslaunch ur5_moveit_config moveit_rviz.launch config:=true` which will bring up an Rviz window. Check that the arm configuration matches what you expect. If it does not then the driver probably did not launch properly, check your terminal windows for errors.

9. You can leave the OMPL planner as 'unspecified' and change to the 'Planning' tab. Click 'Select Start State', make sure it is set to <current> and click 'Update'.

10. Move the end of the arm slightly by dragging the blue ball. Then click the 'Plan' button. Check that the plan is reasonable, will not go through the table and won't destroy your end effector. If it does several ridiculous loops before reaching the goal, you probably didn't set the joint limits to be reduced in the `~/catkin_ws/src/ur_modern_driver/launch/ur5_bringup_LBTF.launch` file.

11. If you are happy with the plan, be ready on the e-stop button and click 'Execute'. 

Congratulations, you now have the arm running with moveit!


# Using ros_control

An alternative method for control is using the ros_control package. To launch this by default you need to change the ur5_ros_control.launch file slightly. (Or the appropriate launch file for your robot if using UR10 or UR3).

For a UR5 you can replace the `~/catkin_ws/src/ur_modern_driver/launch/ur5_ros_control.launch` file with the one from this repo under `ros_control/ur5_ros_control.launch`, the changes to the file are:

`<arg name="controllers" default="joint_state_controller force_torque_sensor_controller vel_based_pos_traj_controller"/>`
`<arg name="stopped_controllers" default="pos_based_pos_traj_controller joint_group_vel_controller"/>`
`<arg name="limited" default="false"/>`

to

`<arg name="controllers" default="joint_state_controller force_torque_sensor_controller pos_based_pos_traj_controller"/>`
`<arg name="stopped_controllers" default="vel_based_pos_traj_controller joint_group_vel_controller"/>`
`<arg name="limited" default="true"/>`

This will now launch the position controller by default, with limited joint angles so that moveit! planning works. You also need to add the position controller to the list of moveit! controllers. Replace the `controllers.yaml` file in `catkin_ws/src/universal_robot/ur5_moveit_config/config` with the one from this repository in the `ros_control/controller.yaml` location. If using UR10 / UR3 make sure the alter the correct controllers file. 

## Launching with ros_control
To actually control the arm once you have made the above changes you need to establish a connection, enable the motors, launch the driver, launch the planner and finally launch Rviz. To do this:

1. Power on the arm, enable the DHCP setting and take note of the IP address once it appears

2. Check you can ping the arm  

3. Open an empty program, enable the arm motors and drive the arm to the home position

4. Reduce the speed as a safety measure. 10-30% speed is a good starting point.

5. You should now be in the 'Move' menu on the UR tablet. If you are in the movement execution window (the one that has an 'Auto' and a 'Manual' button) then the driver will not work.

Run each of the following commands in a different terminal.

6. Run `roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=xxx.xxx.xxx.xxx` using the IP address you found before

7. Run `roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch` and wait for this to read `You can start planning now!`

8. Run `roslaunch ur5_moveit_config moveit_rviz.launch config:=true` which will bring up an Rviz window. Check that the arm configuration matches what you expect. If it does not then the driver probably did not launch properly, check your terminal windows for errors.

9. You can leave the OMPL planner as 'unspecified' and change to the 'Planning' tab. Click 'Select Start State', make sure it is set to <current> and click 'Update'. You need to update the start state after every move if you are running at <100% speed, otherwise moveit! will complain.

10. Move the end of the arm slightly by dragging the blue ball. Then click the 'Plan' button. Check that the plan is reasonable, will not go through the table and won't destroy your end effector. If it does several ridiculous loops before reaching the goal, you probably didn't set the joint limits to be reduced in the `~/catkin_ws/src/ur_modern_driver/launch/ur5_ros_control.launch` file.

11. If you are happy with the plan, be ready on the e-stop button and click 'Execute'. If running at <100% you should observe controller overshoot/undershoot.

Congratulations, you now have the arm running with moveit!


