# Motion planning simulation setup for kuka_kr6 arm
## Overview
This project is written for the graduation program in ZJU. It's a simulation setting for kuka_kr6r900sixx manipulator on gazebo and displayed by rviz. User can use function in Moveit! or write his own motion planning function and test them by this simulation setting.

Plus a Qt-based simulation environment in the folder rrt-simulator-master forked from this repo:  
> https://github.com/sourishg/rrt-simulator  

The repo also includes two new RRT based algorithms which suggests that generating a simple path before RRT exploration would acclerate the speed of path finding. Detailed description could be found in "3 RRT-based algorithm"  

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
├────── Motion planning functions. Method list: 1 Moveit built-in functions; 2 rrt; 3 new rrt(unfinished)  
├──rrt-simulator-master  
└────── A Qt-based rrt simulation forked from https://github.com/sourishg/rrt-simulator with rrtMult newly written algorithm  

## Usage
User currently has three options:  
1 - use moveit built-in motion planning function;  
2 - use RRT function (unfinished);  
3 - new RRT-based algorithm (working)  

### 1 Moveit build-in functions
After source devel file and use rosdep to install required packages, run the following command  
> roslaunch moveit_planning planning_scene.launch  

The gazebo is not displayed to save computation resources. To display the gazebo window, run this command instead:  
> roslaunch moveit_planning planning_scene.launch showGazebo:=true  
  

The rviz and gazebo should be like:  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/rviz_no_planning.png)  
> rviz_no_planning.png  

![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/gazebo_no_plann.png)  
> gazebo_no_planning.png  


In the "Displays" panel in the left upper part of rviz, click the MotionPlanning->PlanningRequest->Query Goal State.  
Drag the interaction marker displayed on the end-effector and click "Plan and Execute" in the MotionPlanning panel.  
To see the path in loop or in trail, click Displays->MotionPlanning->Planned Path->Loop Animation || Show Trail  
The effect in rviz and gazebo should be:  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/rviz_planned.png)  
> rviz_planned.png  

![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/gazebo_planned.png)  
> gazebo_planned.png  



### 2 RRT (not finished)
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
"tolerance" is the angle tolerance to goal in degree.   
> rosrun moveit_planning rrtPlanner visual 1 maxIter 1000  

When terminal prompted that "Waiting to continue, Press 'next' to plan a path", press the 'next' button on the RvizVisualToolsGui panel in rviz.  
For visualization type 1 and maxIter 1000, the effect would be:  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/rrtPlanner_before_next.png)  
> rrtPlanner_combined.png  

For maxIter 10000, the search result is as follows  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/maxIter10000.png)  
> image after 'rosrun moveit_planner rrtPlanner visual 1 maxIter 10000'  

### 3 RRT-based algorithm  
**TODO**  


## Working Pipeline Explanation
**TODO**  

## Qt-based simulation
To run the simulation, open the terminal in the rrt-simulator-master and run  
> ./build.sh  

Note that you need Qt tools to build the target  
Then run:  
> ./bin/rrt-test  

Then please run as the original repository says.

There are three methods implemented in this program. To change methods user need change several variable in the program (still working on it to make it more user-friendly). The basic outcome is as follows:  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/RRT_original.png)  
> Original rrt method  
  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/RRT_sep.png)  
> Seperate and link groups seperately  
  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/RRTMult.png)  
> Seperate the simple path and link them 

## TODO
1 Design the new algorithm and test it in this platform  
2 Implement the new algorithm in the ROS settings  
