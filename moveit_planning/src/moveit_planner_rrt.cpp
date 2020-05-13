#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkStates.h>
#include <math.h>
#include <time.h>
#include <Eigen/Dense>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "obstacle_adder.h"
#include "rrtPlanner.h"

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

void rotateTargetPose(Eigen::Quaterniond q, geometry_msgs::Pose* pose)
{
    // Rotate the target quaternion so that the gripper could grasp the target from top to bottom
    Eigen::Matrix3d tarRot = q.toRotationMatrix();
    // Rotate the matrix, axis: x, angle: 90 degree
    Eigen::Matrix3d rotAxisX;
    rotAxisX << 0, 0, 1,
                0, 1, 0,
                -1, 0, 0;
    // rotAxisX << 1,0,0,
    //             0,1,0,
    //             0,0,1;
    // Get the target pose
    Eigen::Matrix3d result = rotAxisX * tarRot;
    Eigen::Matrix3d qMat;
    qMat << result(0,0), result(0,1), result(0,2),
            result(1,0), result(1,1), result(1,2),
            result(2,0), result(2,1), result(2,2);
    Eigen::Quaterniond qResult;
    qResult = qMat;
    pose->orientation.x = qResult.x();
    pose->orientation.y = qResult.y();
    pose->orientation.z = qResult.z();
    pose->orientation.w = qResult.w();
    cout << "The final quaternion is:\n" << qResult.x() << " " << qResult.y() << " " << qResult.z() << " " << qResult.w() << endl;
}

geometry_msgs::Pose setTarget(tf::StampedTransform targetTrans)
{
    geometry_msgs::Pose target_pose;
    // Get the rotation
    Eigen::Quaterniond q(targetTrans.getRotation().x(),
                         targetTrans.getRotation().y(),
                         targetTrans.getRotation().z(),
                         targetTrans.getRotation().w());
    cout << "The rotation of target is: \n" << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl; 
    rotateTargetPose(q, &target_pose);

    // Get the translation
    target_pose.position.x = targetTrans.getOrigin().x();
    target_pose.position.y = targetTrans.getOrigin().y();
    target_pose.position.z = targetTrans.getOrigin().z()+0.04;
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
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader_));

    // Initialize the obstacleAdder and rrtPlanner
    Obstacle_Adder obs_adder = Obstacle_Adder(nh);
    rrtPlanner rrt_planner = rrtPlanner(nh, kinematic_model, planning_scene_monitor_);
    if(argc>1){
        rrt_planner.getParamFromCommandline(argc, argv);
    }

    // Start moveit visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl(); 
    visual_tools.trigger();

    // MAIN LOOP
    ROS_INFO("Currently running moveit planning");
    while(ros::ok()){
        // Refresh the obstacles for next plan
        obs_adder.add_obstacles();

        visual_tools.prompt("Press 'next' to plan a path");

        // Listen to tf broadcaster and set the transform as the move_group target
        tf::StampedTransform transform = getTargetTrans();
        tf::Quaternion q = transform.getRotation();

        // Check whether receive any transform
        if(pow(q[0],2) + pow(q[1],2) + pow(q[2],2) + pow(q[3],2) == 1){   
            ROS_INFO("Entering the planning mode");     
            geometry_msgs::Pose target_pose = setTarget(transform);
            rrt_planner.setGoalNodeFromPose(target_pose);

            ROS_INFO("The target_pose is received");
            cout << "The orientation are: " << target_pose.orientation.x << " " << target_pose.orientation.y << " " << target_pose.orientation.z << " " << target_pose.orientation.w << endl;
            cout << "The translations are: " << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << endl;        

            rrt_planner.setInitialNode(move_group.getCurrentJointValues());
            // Only for debugging, comment this line when running
            // vector<double> jointPosition{0,0.9,0,0,0,0};
            // rrt_planner.setInitialNode(jointPosition);

            clock_t start, finish;
            start = clock();
            bool success = rrt_planner.plan();
            finish = clock();
            double time = (double)(finish-start)/CLOCKS_PER_SEC;

            if(success){
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                rrt_planner.generatePlanMsg(time, &my_plan);

                cout << "Trying to get robot_state trajectory" << endl;
                ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
                visual_tools.publishAxisLabeled(target_pose, "pose1");
                visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
                visual_tools.trigger();
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
