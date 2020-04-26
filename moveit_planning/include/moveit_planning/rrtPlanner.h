#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std;

#define JOINTNUM 6

struct node
{
    int id;
    int jointAngles[JOINTNUM];
    int prevNodeid;
    int nextNodeid;
};

class rrtPlanner
{
public:
    rrtPlanner(ros::NodeHandle& nh, geometry_msgs::Pose objPose);
    void generateGraspPose(geometry_msgs::Pose objPose);
    node sampleNode();
    int findNearest(node randNode);
    double calcDist(node a, node b);
    void extend(int id, node randNode);
    bool checkReachGoal(node newNode);
    bool checkFeasbility(node nearestNode);
    void findPath();
    bool plan();

    double GOALTOLERANCE = 0.05;
    double STEP = 20;
    bool success = false;
    ros::NodeHandle& nh;
    vector<node> rrtTree;
    vector<node> path;
    vector<node> goalNodes;
    int jointUpperLimits[JOINTNUM];
    int jointLowerLimits[JOINTNUM];
};

#endif