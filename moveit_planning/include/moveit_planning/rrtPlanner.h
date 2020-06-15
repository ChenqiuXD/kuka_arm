#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include <ros/ros.h>
#include <float.h>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/RobotState.h>

using namespace std;

#define JOINTNUM 6
#define PI2DEG 180/3.14159
#define DEG2PI 3.1415926/180

struct node
{
    int id;
    vector<int> jointAngles;
    int prevNodeid;
    int cluster_id;
};

class rrtPlanner
{
public:
    // rrtPlanner(){;}
    rrtPlanner(ros::NodeHandle& nh,
               robot_model::RobotModelPtr kinematic_model,
               planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
    
    // RRT core functions
    node sampleNode();
    int findNearest(node randNode, vector<node> tree);
    double calcDist(node a, node b);
    node extend(int id, node randNode);
    bool checkReachGoal(node newNode);
    bool checkFeasbility(node);
    bool checkFeasbility(node nearestNode, node newNode);
    void findPath();
    bool plan();

    // RRT utils functions
    void readJntLimits();
    void setGoalNode(vector<int> jointPosition);
    void setGoalNode(vector<double> jointPosition);
    void setGoalNodeFromPose(geometry_msgs::Pose objPose);
    void setInitialNode(vector<int> jointPoses);
    void setInitialNode(vector<double> jointPoses);
    void initialize();
    void initrrtVisual();
    void initPathVisual();
    void drawNewNode(node newNode);
    void drawPlan();
    void calcNodePose(node newNode, geometry_msgs::Point *nodePose);
    void generatePlanMsg(double time, moveit::planning_interface::MoveGroupInterface::Plan* my_plan);

    void degreeToRadian(vector<int> degree, vector<double> *radian);
    void radianToDegree(vector<double> radian, vector<int> *degree);
    void getParamFromCommandline(int argc, char** argv);
    void setParam(string paramName, string paramValue);
    void setVisualParam(int visualType);
    void vecInt2Double(vector<int>, vector<double> *);
    void vecDoub2Int(vector<double> , vector<int> *);

    // Angle tolerance per joint
    double goalTolerance = 3;
    vector<double> goalToleranceVec{3,3,3,360,360,360};
    // STEP : Used in extend, determine the extending step
    double STEP = 0.5;
    // FEASI_PIESCES_NUM : Used in checkFeasibility, determine the number of pieces
    // between nearestNode and newNode to be check collision
    int FEASI_PIESCES_NUM = 3; 
    // In plan, the maximum of extend iteration
    int maxIter = 3000;
    // Name of the end-effctor
    string END_EFFECTOR_NAME = "tool0";
    // With probability the tree would extend towards the goal
    bool goalExtend = true;

    robot_model::RobotModelPtr kinematicModel;
    robot_state::RobotStatePtr kinematicState;
    robot_state::JointModelGroup* jointModelGroup;
    planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor_;
    ros::NodeHandle& nh;
    ros::Publisher markerPub;
    // rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    visualization_msgs::Marker points;
    visualization_msgs::Marker line_list;
    visualization_msgs::Marker pathVertices;
    visualization_msgs::Marker pathEdges;
    geometry_msgs::Pose goalPose;

    double minGoalDist = DBL_MAX;
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