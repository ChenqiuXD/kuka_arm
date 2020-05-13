#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
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
               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
    rrtPlanner(ros::NodeHandle& nh,
               robot_model::RobotModelPtr kinematic_model,
               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
    void generateGraspPose(geometry_msgs::Pose objPose);
    void setInitialNode(vector<int> jointPoses);
    void setInitialNode(vector<double> jointPoses);
    void initialize();
    void initrrtVisual();
    void initPathVisual();
    void setVisualParam(int visualType);
    node sampleNode();
    int findNearest(node randNode);
    double calcDist(node a, node b);
    void extend(int id, node randNode);
    void drawNewNode(node newNode);
    void calcNodePose(node newNode, geometry_msgs::Point *nodePose);
    bool checkReachGoal(node newNode);
    bool checkFeasbility(node nearestNode, node newNode);
    void findPath();
    void generatePlan(double time, moveit::planning_interface::MoveGroupInterface::Plan* my_plan);
    bool plan();

    // util functions
    void degreeToRadian(vector<int> degree, vector<double> *radian);
    void radianToDegree(vector<double> radian, vector<int> *degree);
    void getParamFromCommandline(int argc, char** argv);
    void setParam(string paramName, string paramValue);

    // Angle tolerance per joint
    double GOALTOLERANCE = 3;
    // STEP : Used in extend, determine the extending step
    double STEP = 0.5;
    // FEASI_PIESCES_NUM : Used in checkFeasibility, determine the number of pieces
    // between nearestNode and newNode to be check collision
    int FEASI_PIESCES_NUM = 1; 
    // In plan, the maximum of extend iteration
    int MAXITER = 4500;
    // Name of the end-effctor
    string END_EFFECTOR_NAME = "tool0";

    robot_model::RobotModelPtr kinematicModel;
    robot_state::RobotStatePtr kinematicState;
    robot_state::JointModelGroup* jointModelGroup;
    planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor_;
    ros::NodeHandle& nh;
    ros::Publisher markerPub;
    visualization_msgs::Marker points;
    visualization_msgs::Marker line_list;

    double minGoalDist = float('inf');
    double maxGoalDist = 0;
    bool success = false;
    node initialNode;
    int enableVisual = 1;
    enum VISUAL_TYPES { NO_VISUAL, VISUAL_ALL, VISUAL_STEP };
    vector<node> rrtTree;
    vector<node> path;
    vector<node> goalNodes;
    int jointUpperLimits[JOINTNUM];
    int jointLowerLimits[JOINTNUM];
};

#endif