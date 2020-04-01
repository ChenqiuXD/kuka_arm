## Overview
This project is written for the graduation program in ZJU. One can use gazebo to simulate while utilising motion planning functions in MoveIt! package.

## Usage
To run the simulation, use the following two commands after catkin_make in the top level.
roslaunch kuka_kr6_gazebo kuka_kr6_gazebo.launch
roslaunch kuka_kr6r900sixx_moveit_config moveit_planning_execution.launch

You should see a kuka_arm spawned in an empty world with a kinect camera in the air representing by a red cylinder. To use the moveit's motion planning method, add 'motion planning' panel in rviz and update a new goal. Press the 'plan and execute' and observe that in both rviz and gazebo, the kuka arm moved with command. 

To visulaize the image obtained from kinect, run
rosrun image_view image_view image:='camera/color/image_raw'

## TODO
Use some algorithm to get the position of the target object
Learn about the grasp interface of moveit.