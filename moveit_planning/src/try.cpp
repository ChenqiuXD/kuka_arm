#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>

#include <vector>

using namespace std;

void getRobotTraj(vector<moveit_msgs::RobotTrajectory>* trajectory)
{
    moveit_msgs::RobotTrajectory robot_trajectory;
    trajectory_msgs::JointTrajectory joint_trajectory;
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = "/world";

    int wayPointNum = 10;
    vector<trajectory_msgs::JointTrajectoryPoint> wayPoints;
    for(size_t i=0;i<wayPointNum;i++){
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = {i*0.1, i*0.1, i*0.1, i*0.1, i*0.1, i*0.1};
        // point.time_from_start = 0.1*i;
        wayPoints.push_back(point);
    }

    joint_trajectory.header = header;
    joint_trajectory.joint_names = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};
    joint_trajectory.points = wayPoints;

    robot_trajectory.joint_trajectory = joint_trajectory;
    trajectory->push_back(robot_trajectory);
}

void getTrajectoryStart(moveit_msgs::RobotState* trajectory_start)
{
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = "/world";

    sensor_msgs::JointState joint_state;
    joint_state.header = header;
    joint_state.name = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"};
    joint_state.position = {0,0,0,0,0,0};
    joint_state.velocity = {0,0,0,0,0,0};
    joint_state.effort = {0,0,0,0,0,0};

    trajectory_start->joint_state = joint_state;
}

void getStatus(actionlib_msgs::GoalStatusArray* status)
{
    std_msgs::Header header;
    header.seq = 0;
    header.frame_id = "/world";
    status->header = header;

    actionlib_msgs::GoalStatus goal_status;
    goal_status.status = goal_status.SUCCEEDED;
    actionlib_msgs::GoalID goal_id;
    goal_id.id = "moveit_rviz";
    goal_status.goal_id = goal_id;
    goal_status.text = "Motion plan was computed succesfully";
    status->status_list.push_back(goal_status);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pos_listener");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher trajPub = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1);
    ros::Publisher statusPub = nh.advertise<actionlib_msgs::GoalStatusArray>("/move_group/status", 1);

    moveit_msgs::DisplayTrajectory traj;
    traj.model_id = "kuka_kr6";    
    getRobotTraj(&traj.trajectory);
    getTrajectoryStart(&traj.trajectory_start);
    
    actionlib_msgs::GoalStatusArray status;
    getStatus(&status);

    ros::Rate loop_rate(30);
    while(ros::ok()){
        trajPub.publish(traj);
        statusPub.publish(status);
        loop_rate.sleep();
    }
    
    return 0;
}