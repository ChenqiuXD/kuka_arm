#ifndef SEP_RRT_PLANNER_H
#define SEP_RRT_PLANNER_H

#include "rrtPlanner.h"

class sep_rrtPlanner : public rrtPlanner
{
    public:
        sep_rrtPlanner(ros::NodeHandle& nh,
                       robot_model::RobotModelPtr kinematic_model,
                       planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
        
        bool plan();
        void initialize();
        void getSimplePath();
        void seperatePaths();
        void seperateSimplePath(vector<node> &path);
        void addInTree(int nodeGroupid);
        void checkConnection(node newNode, int *groupid, int *nodeid);
        void connectToGroup(node *newNode, int groupid, int nodeid);
        void findPath();
        bool chechReachGoal(int groupid);

        // utils
        double getDistStartToEnd();
        void drawSimplePath();
        void drawSimplePathContinuity();
        void drawPlan();
        void initVisual();

        int groupCount;
        vector< vector<node> > simplePaths;
        vector< vector<node> > nodeGroups;
        vector<int> goalGroupCount;
        vector<int> startGroupid;

        visualization_msgs::Marker points_simple;
        visualization_msgs::Marker line_list_simple;

        int EXPAND_RATE = 8;       // Used when generating 
};

#endif