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

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

using namespace std;

class Obstacle_Adder
{
public:
    Obstacle_Adder(ros::NodeHandle& nh, moveit::planning_interface::PlanningSceneInterface* pli);
    ~Obstacle_Adder();
    void obs_callback(const gazebo_msgs::LinkStates::ConstPtr &msg);
    void add_obstacles();

    map<string, geometry_msgs::Pose>obs_poses;
    moveit::planning_interface::PlanningSceneInterface* plan_scene;
    vector<string> obs_names;
    ros::NodeHandle& nh;
    ros::Subscriber link_states_sub;
};

#endif