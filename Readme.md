# Motion planning simulation setup for kuka_kr6 arm
## Overview
This project is written for the bachelor degree thesis in ZJU. The package contains a motion-planning algorithm based on RRT and a simulation setting for kuka_kr6r900sixx manipulator on gazebo and displayed by rviz. The algorithm proposes a pre-plan phase which generates simple paths to accelerate algorithm in the second planning phase. Simulation proved that the the new algorithm has bigger success rate and uses shorter time than other RRT-based motion planning algorithms.  

The package also contains a Qt-based simulation environment in the folder rrt-simulator-master forked from this repo:  
> https://github.com/sourishg/rrt-simulator  

## Packages and brief introduction
src  
├── kuka_kr6_control  
├────── Controller for simulation in gazebo  
├── kuka_kr6_description  
├────── urdf files for kuka_kr6 arm, camera, etc  
├── kuka_kr6_gazebo  
├────── Models and worlds file used in gazebo  
├── kuka_kr6r900sixx_moveit_config  
├────── Auto generated moveit_config_package by Moveit! using the urdf file in kuka_kr6_description.  
├────── Check for the tutorials about connecting rviz and gazebo for detailed description  
├── moveit_planning  
├────── Motion planning functions. Method list: 1 Moveit built-in functions; 2 rrt; 3 new rrt  
├── rrt-simulator-master  
└────── A Qt-based rrt simulation forked from https://github.com/sourishg/rrt-simulator with rrtMult newly written algorithm  

## Usage
User currently has three options:  
1 - use moveit built-in motion planning function;  
2 - use RRT function;  
3 - new RRT-based algorithm;

P.S. If your gazebo could not find models, please add the .../catkin_ws/src/kuka_kr6_gazebo/models to your gazebo models directory lists ('...' should be your path). By editing the ~/.gazebo/gui.ini. Or in the terminal add this path to ${GAZEBO_MODEL_PATH}
> export GAZEBO_MODEL_PATH=$HOME/.../catkin_ws/src/kuka_kr6_gazebo/models:$GAZEBO_MODEL_PATH  

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

### 2 RRT
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
> result of command 'rosrun moveit_planner rrtPlanner visual 1 maxIter 10000'  

### 3 RRT-based algorithm  
To run different RRT-based algorithm, run the following command:  
> roslaunch moveit_planning planning_scene.launch useRRT:=true  
> rosrun moveit_planning imgProcess.py  
> rosrun moveit_planning rrtPlanner visual 1 maxIter 1000  rrtType 1  

Here 'rrtType': 0 -> RRT, 1 -> bi-directional RRT, 2 -> RRT-Connect


## Methods Explanation
All the planning codes are in the moveit_planning package.  

| Names | methods |  
-|:-:|-:  
|rrtPlanner|Traditional RRT planner|  
|bi_rrtPlanner|Bi-directional RRT planner|   
|rrtConnectPlanner|RRT-Connect|  
|rrtMult|Proposed method|  
|bi_rrtMultPlanner|Proposed method combined with bi-directional RRT|  
|rrtMultConnect|Proposed method combined with RRT-Connec|  

The proposed method basically assume that a pre-planned phase could accelerate the whole planning process. The algorithm would generate several simple paths in the first stage and would further check the feasibility of these paths. By eliminating colliding segments, the first stage generate several feasible sub-paths. In the second phase, algorithm would try to connect those sub-paths to reach the goal.  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/***.png)  
> A sketch of planning process

Simulation results prove that this algorithm could achieve higher success rate and shorter planning time compared with RRT. Besides, this algorithm could easily combined with RRT variant algorithms and achieve better results.  
![Alt text](https://github.com/ChenqiuXD/kuka_arm/blob/master/images/***.png)  
> The comparision between different planning algorithms in ROS environment  

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

