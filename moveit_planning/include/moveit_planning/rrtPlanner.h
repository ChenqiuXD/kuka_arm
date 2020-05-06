#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Geometry>
#include <Eigen/Core>

using namespace std;

#define JOINTNUM 6
#define PI2DEG 180/3.14159
#define DEG2PI 3.1415926/180

struct node
{
    int id;
    vector<int> jointAngles;
    int prevNodeid;
};

class rrtPlanner
{
public:
    rrtPlanner(ros::NodeHandle& nh, geometry_msgs::Pose objPose,
               robot_model::RobotModelPtr kinematic_model,
               planning_scene::PlanningScene* planning_scene);
    rrtPlanner(ros::NodeHandle& nh,
               robot_model::RobotModelPtr kinematic_model,
               planning_scene::PlanningScene* planning_scene);
    void generateGraspPose(geometry_msgs::Pose objPose);
    void setInitialNode(vector<int> jointPoses);
    void setInitialNode(vector<double> jointPoses);
    void initialize();
    node sampleNode();
    int findNearest(node randNode);
    double calcDist(node a, node b);
    void extend(int id, node randNode);
    bool checkReachGoal(node newNode);
    bool checkFeasbility(node nearestNode, node newNode);
    void findPath();
    void generatePlan(double time, moveit::planning_interface::MoveGroupInterface::Plan* my_plan);
    bool plan();

    // Angle tolerance per joint
    double GOALTOLERANCE = 5;
    // STEP : Used in extend, determine the extending step
    double STEP = 0.5;
    // FEASI_PIESCES_NUM : Used in checkFeasibility, determine the number of pieces
    // between nearestNode and newNode to be check collision
    int FEASI_PIESCES_NUM = 4; 
    // In plan, the maximum of extend iteration
    int MAXITER = 200;

    robot_model::RobotModelPtr kinematicModel;
    robot_state::RobotStatePtr kinematicState;
    robot_state::JointModelGroup* jointModelGroup;
    planning_scene::PlanningScene* planningScene;
    ros::NodeHandle& nh;

    double minGoalDist = float('inf');
    double maxGoalDist = 0;
    bool success = false;
    node initialNode;
    vector<node> rrtTree;
    vector<node> path;
    vector<node> goalNodes;
    int jointUpperLimits[JOINTNUM];
    int jointLowerLimits[JOINTNUM];
};

#endif