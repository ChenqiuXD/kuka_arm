#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <math.h>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "obstacle_adder.h"

using namespace std;

tf::StampedTransform getTargetTrans()
{
    tf::TransformListener listener;
    int TF_TIMEOUT = 1;

    // Listen to the tf message to get the target pose transform published by imgProcess node
    tf::StampedTransform transform;
    listener.waitForTransform("/base_link", "/target_pos", ros::Time(0), ros::Duration(TF_TIMEOUT*5), ros::Duration(TF_TIMEOUT/3));
    try{
        listener.lookupTransform("/base_link", "/target_pos", ros::Time(0), transform);
        ROS_INFO("Successfully listen from the tf listener");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return transform;
}

geometry_msgs::Pose setTarget(tf::StampedTransform targetTrans)
{
    geometry_msgs::Pose target_pose;
    // target_pose.orientation.x = transform.getRotation().x();
    // target_pose.orientation.y = transform.getRotation().y();
    // target_pose.orientation.z = transform.getRotation().z();
    // target_pose.orientation.w = transform.getRotation().w();
    target_pose.orientation.w = 1;
    target_pose.position.x = targetTrans.getOrigin().x();
    target_pose.position.y = targetTrans.getOrigin().y();
    target_pose.position.z = targetTrans.getOrigin().z();
    target_pose.position.z += 0.3;

    // cout << target_pose.orientation.x << target_pose.orientation.y << target_pose.orientation.z << target_pose.orientation.w << endl;
    return target_pose;
}

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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Initialize the obstacle adder to add obstacle from gazebo
    // Obstacle_Adder obs_adder = Obstacle_Adder(nh, &planning_scene_interface);

    // Start moveit visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl(); 
    visual_tools.trigger();

    // MAIN LOOP
    ROS_INFO("Currently running moveit planning");
    while(ros::ok()){
        // Refreshe the obstacles for next plan
        // planning_scene_interface.world.collision_objects.clear();
        // obs_adder.add_obstacles();

        visual_tools.prompt("Press 'next' to plan a path");

        // Listen to tf broadcaster and set the transform as the move_group target
        tf::StampedTransform transform = getTargetTrans();
        tf::Quaternion q = transform.getRotation();
        move_group.setGoalTolerance(0.05);
        // If did not get any of the target transform, the 2-norm of quaernion would not equals 1
        if(pow(q[0],2) + pow(q[1],2) + pow(q[2],2) + pow(q[3],2) == 1){   
            ROS_INFO("Entering the planning mode");     
            geometry_msgs::Pose target_pose = setTarget(transform);
            move_group.setPoseTarget(target_pose);
            // if(iter%3=0){add_obstacles(nh);}

            ROS_INFO("The target_pose is received");
            cout << "The orientation are: " << target_pose.orientation.x << " " << target_pose.orientation.y << " " << target_pose.orientation.z << " " << target_pose.orientation.w << endl;
            cout << "The translations are: " << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << endl;        

            // Plan the path
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

            ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
            visual_tools.publishAxisLabeled(target_pose, "pose1");
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
            visual_tools.trigger();
        }else{
            ROS_ERROR("No target transform is received, please check the output of imgProcess.py");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
