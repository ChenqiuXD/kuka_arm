#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <math.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "target_pose_utils.h"
#include "obstacle_adder.h"

using namespace std;

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "pos_listener");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(30);

    // Initialize moveit relevent variables
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    // robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    // robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    // planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

    // Initialize the obstacle adder to add obstacle from gazebo
    Obstacle_Adder obs_adder = Obstacle_Adder(nh);

    // Start moveit visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl(); 
    visual_tools.trigger();

    // MAIN LOOP
    ROS_INFO("Currently running moveit planning");
    while(ros::ok()){
        visual_tools.prompt("Press 'next' to plan a path");
        // Refresh the obstacles for next plan
        // obs_adder.add_obstacles();
        // obs_adder.generateFixedObs();
        obs_adder.generateRandomObs();

        // Listen to tf broadcaster and set the transform as the move_group target
        tf::StampedTransform transform = getTargetTrans();
        tf::Quaternion q = transform.getRotation();
        move_group.setGoalTolerance(0.005);
        // if receive any target translation
        if(pow(q[0],2) + pow(q[1],2) + pow(q[2],2) + pow(q[3],2) == 1){   
            ROS_INFO("Entering the planning mode");     
            geometry_msgs::Pose target_pose = setTarget(transform);
            move_group.setPoseTarget(target_pose);

            ROS_INFO("The target_pose is received");
            cout << "The orientation are: " << target_pose.orientation.x << " " << target_pose.orientation.y << " " << target_pose.orientation.z << " " << target_pose.orientation.w << endl;
            cout << "The translations are: " << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << endl;        

            // Plan the path
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(success){move_group.execute(my_plan);}
            // You can also use move_group.move(), which would plan and execute.

            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
        }else{
            ROS_ERROR("No target transform is received, please check the output of imgProcess.py");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
