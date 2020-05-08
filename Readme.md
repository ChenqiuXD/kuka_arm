# Motion planning simulation setup for kuka_kr6 arm
## Overview
This project is written for the graduation program in ZJU. It's a simulation setting for kuka_kr6r900sixx manipulator on gazebo and displayed by rviz. User can use function in Moveit! or write his own motion planning function and test them by this simulation setting.

## Packages and brief introduction
src  
├── kuka_kr6_control  
├────── Controller for simulation in gazebo  
├── kuka_kr6_description  
├────── Urdf files for kuka_kr6 arm, camera, etc  
├── kuka_kr6_gazebo  
├────── Models and worlds file used in gazebo  
├── kuka_kr6r900sixx_moveit_config  
├────── Auto generated moveit_config_package by Moveit! using the urdf file in kuka_kr6_description.  
├────── Check for the tutorials about connecting rviz and gazebo for detailed description  
├── moveit_planning  
└────── Motion planning functions. Method list: 1 Moveit built-in functions; 2 rrt; 3 new rrt(unfinished)  

## Usage
User currently has three options:  
1 - use moveit built-in motion planning function;  
2 - use RRT function written by arthur (unfinished);  
3 - Learned RRT (not started)  

### 1 Moveit build-in functions
After source devel file and use rosdep to install required packages, run the following command  
> roslaunch moveit_planning planning_scene.launch  

The gazebo is not displayed to save computation resources. To display the gazebo window, run this command instead:  
> roslaunch moveit_planning planning_scene.launch showGazebo:=true  
  

The rviz and gazebo should be like:  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/rviz_no_planning.png)  
> rviz_no_planning.png  

![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/gazebo_no_plann.png)  
> gazebo_no_plann.png  


In the "Displays" panel in the left upper part of rviz, click the MotionPlanning->PlanningRequest->Query Goal State.  
Drag the interaction marker displayed on the end-effector and click "Plan and Execute" in the MotionPlanning panel.  
To see the path in loop or in trail, click Displays->MotionPlanning->Planned Path->Loop Animation || Show Trail  
The effect in rviz and gazebo should be:  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/rviz_planned.png)  
> rviz_planned.png  

![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/gazebo_planned.png)  
> gazebo_planned.png  



### 2 RRT functions written by arthur (not finished)
After source and install required packages, run following command, to show the gazebo, add the showGazebo param like above.  
> roslaunch moveit_planning planning_scene.launch useRRT:=true  

Then run following command so to detect the target object and publish tf message   
> rosrun moveit_planning imgProcess.py 
  
  
A picture should appear and tf message would be displayed in rviz:   
(The target_pos tf might be jerking. Guess the problem lies in the model. Still working on it)  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/imgProcess.png)  
> imgProcess.png  


Start the rrt functions and assign the maximum node counts and visualization options:  
visulization_options:  
├── 0 : no visulization  
├── 1 : visualise the vertices and edges  
└── 2 : visualise the generation of vertices per sec  
"maxIter" is an int and thus should not exceed int range.  
> rosrun moveit_planning rrtPlanner visual 1 maxIter 1000  

When terminal prompted that "Waiting to continue, Press 'next' to plan a path", press the 'next' button on the RvizVisualToolsGui panel in rviz.  
For visualization type 1 and maxIter 1000, the effect would be:  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/rrtPlanner_before_next.png)  
> rrtPlanner_combined.png  



### 3 Learnt RRT
**TODO**



## Working Pipeline Explanation
**TODO**



## TODO
1 RRT is not finished therefore the outcome is incorrect  
2 Design the new algorithm and test it in this platform  
