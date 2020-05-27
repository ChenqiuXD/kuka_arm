#ifndef OBSTACLE_ADDER_H
#define OBSTACLE_ADDER_H

#include <ros/ros.h>
#include <string.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <vector>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

using namespace std;

class Obstacle_Adder
{
public:
    Obstacle_Adder(ros::NodeHandle& nh);
    ~Obstacle_Adder();
    void obs_callback(const gazebo_msgs::LinkStates::ConstPtr &msg);
    void add_obstacles();
    moveit_msgs::CollisionObject add_wooden_box();
    moveit_msgs::CollisionObject add_target();
    moveit_msgs::CollisionObject add_test_obstacles();

    map<string, geometry_msgs::Pose>obs_poses;
    vector<string> obs_names;
    ros::NodeHandle& nh;
    ros::Subscriber link_states_sub;
    ros::Publisher obstacle_pub;
};

#endif