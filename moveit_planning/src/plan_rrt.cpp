#include <ros/ros.h>
#include <string.h>
#include <gazebo_msgs/LinkStates.h>
#include <math.h>
#include <time.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "target_pose_utils.h"
#include "obstacle_adder.h"
#include "rrtPlanner.h"
#include "bi_rrtPlanner.h"
#include "rrtConnectPlanner.h"
#include "sep_rrtPlanner.h"
#include "bi_rrtMultPlanner.h"
#include "rrtMultConnect.h"

using namespace std;

int getrrtType(int argc, char** argv)
{
    int rrtTypeResult = 3;
    for(int i=1;i<argc;++i){
        string paramName = argv[i];
        string paramValue = argv[++i];
        if(paramName=="rrtType"){
            if(paramValue=="0"){rrtTypeResult = 0;}         // Original RRT
            else if(paramValue=="1"){rrtTypeResult = 1;}    // Bi-directional RRT
            else if(paramValue=="2"){rrtTypeResult = 2;}    // RRT-Connect
            else if(paramValue=="3"){rrtTypeResult = 3;}    // Seperate RRT
            else if(paramValue=="4"){rrtTypeResult = 4;}    // Bi-RRTMult
            else if(paramValue=="5"){rrtTypeResult = 5;}    // rrtMultConnect
            else{ROS_ERROR("Invalid rrt type, recheck your command input");}
        }
    }
    return rrtTypeResult;
}

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(30);

    // Initialize moveit relevent variables
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

    // Initialize the obstacleAdder and rrtPlanner
    Obstacle_Adder obs_adder = Obstacle_Adder(nh);
    int rrtType =  getrrtType(argc, argv);
    rrtPlanner rrt_planner = rrtPlanner(nh, kinematic_model, planning_scene_monitor_);
    rrt_planner.getParamFromCommandline(argc, argv);
    bi_rrtPlanner bi_rrt_planner = bi_rrtPlanner(nh, kinematic_model, planning_scene_monitor_);
    bi_rrt_planner.getParamFromCommandline(argc, argv);
    rrtConnectPlanner rrt_connect_planner = rrtConnectPlanner(nh, kinematic_model, planning_scene_monitor_);
    rrt_connect_planner.getParamFromCommandline(argc, argv);
    sep_rrtPlanner sep_rrt_planner = sep_rrtPlanner(nh, kinematic_model, planning_scene_monitor_);
    sep_rrt_planner.getParamFromCommandline(argc, argv);
    bi_rrtMultPlanner bi_rrt_mult_planner = bi_rrtMultPlanner(nh, kinematic_model, planning_scene_monitor_);
    bi_rrt_mult_planner.getParamFromCommandline(argc, argv);
    rrtMultConnectPlanner rrt_mult_connect_planner = rrtMultConnectPlanner(nh, kinematic_model, planning_scene_monitor_);
    rrt_mult_connect_planner.getParamFromCommandline(argc, argv);

    // Start moveit visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl(); 
    visual_tools.trigger();

    // MAIN LOOP
    ROS_INFO("Currently running RRT planning");
    while(ros::ok()){
        visual_tools.prompt("Press 'next' to plan a path");
        // Refresh the obstacles for next plan
        obs_adder.add_obstacles();
        // obs_adder.generateFixedObs();
        obs_adder.generateRandomObs();
        visual_tools.deleteAllMarkers();

        // Listen to tf broadcaster and set the transform as the move_group target
        tf::StampedTransform transform = getTargetTrans();
        tf::Quaternion q = transform.getRotation();

        // Check whether receive any transform
        if( abs(pow(q[0],2) + pow(q[1],2) + pow(q[2],2) + pow(q[3],2) - 1)<=0.01) {   
            ROS_INFO("Entering the planning mode");     
            geometry_msgs::Pose target_pose = setTarget(transform);

            ROS_INFO("The target_pose is received");
            cout << "The orientation are: " << target_pose.orientation.x << " " << target_pose.orientation.y << " " << target_pose.orientation.z << " " << target_pose.orientation.w << endl;
            cout << "The translations are: " << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << endl;        

            vector<double> jointPosition{-0.170868, 0.592920, 0.298936, -0.000792, 0.042167, 0};
            bool isGoalBlocked;
            if(rrtType==0){
                // rrt_planner.setGoalNodeFromPose(target_pose);
                rrt_planner.setGoalNode(jointPosition);
                rrt_planner.setInitialNode(move_group.getCurrentJointValues());
                isGoalBlocked = !rrt_planner.checkFeasbility(rrt_planner.goalNodes[0]);
            }else if(rrtType==1){
                // bi_rrt_planner.setGoalNodeFromPose(target_pose);
                bi_rrt_planner.setGoalNode(jointPosition);
                bi_rrt_planner.setInitialNode(move_group.getCurrentJointValues());
                isGoalBlocked = !bi_rrt_planner.checkFeasbility(bi_rrt_planner.goalNodes[0]);
            }else if(rrtType==2){
                // rrt_connect_planner.setGoalNodeFromPose(target_pose);
                rrt_connect_planner.setGoalNode(jointPosition);
                rrt_connect_planner.setInitialNode(move_group.getCurrentJointValues());
                isGoalBlocked = !rrt_connect_planner.checkFeasbility(rrt_connect_planner.goalNodes[0]);
            }else if(rrtType==3){
                // sep_rrt_planner.setGoalNodeFromPose(target_pose);
                sep_rrt_planner.setGoalNode(jointPosition);
                sep_rrt_planner.setInitialNode(move_group.getCurrentJointValues());
                isGoalBlocked = !sep_rrt_planner.checkFeasbility(sep_rrt_planner.goalNodes[0]);
            }else if(rrtType==4){
                bi_rrt_mult_planner.setGoalNode(jointPosition);
                bi_rrt_mult_planner.setInitialNode(move_group.getCurrentJointValues());
                isGoalBlocked = !bi_rrt_mult_planner.checkFeasbility(bi_rrt_mult_planner.goalNodes[0]);
            }else if(rrtType==5){
                rrt_mult_connect_planner.setGoalNode(jointPosition);
                rrt_mult_connect_planner.setInitialNode(move_group.getCurrentJointValues());
                isGoalBlocked = !rrt_mult_connect_planner.checkFeasbility(rrt_mult_connect_planner.goalNodes[0]);
            }
            
            bool success = false;
            double time;
            if(isGoalBlocked){
                ROS_WARN("The goal is blocked, no availble path could be found");
            }else{
                ROS_INFO("The goal is not in collision, start planning");
                clock_t start, finish;
                start = clock();
                if(rrtType==0){success = rrt_planner.plan();}
                else if(rrtType==1){success = bi_rrt_planner.plan();}     
                else if(rrtType==2){success = rrt_connect_planner.plan();}
                else if(rrtType==3){success = sep_rrt_planner.plan();}
                else if(rrtType==4){success = bi_rrt_mult_planner.plan();} 
                else if(rrtType==5){success = rrt_mult_connect_planner.plan();}      
                finish = clock();
                time = (double)(finish-start)/CLOCKS_PER_SEC;
            }

            if(success){
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                // rrt_planner.generatePlanMsg(time, &my_plan);
                cout << "After " << time << " secs of search, successfully found plan." << endl;
            }else{
                cout << "After " << time << " secs of search, no available plan is found." << endl;
            }

            ROS_INFO_NAMED("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        }else{
            ROS_ERROR("No target transform is received, please check the output of imgProcess.py");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
