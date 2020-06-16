#ifndef RRT_MULT_CONNECT_PLANNER_H
#define RRT_MULT_CONNECT_PLANNER_H

#include "rrtConnectPlanner.h"

class rrtMultConnectPlanner : public rrtConnectPlanner
{
    public:
        rrtMultConnectPlanner(ros::NodeHandle& nh,
                              robot_model::RobotModelPtr kinematic_model,
                              planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
        
        // Core functions
        bool plan();
        void getSimplePath();
        void seperateSimplePaths();
        void seperatePath(vector<node> &path);
        void addToTree();

        // util functions
        double getDistStartToEnd();

        // Member
        int groupCount;
        vector< vector<node> > simplePaths;
        vector< vector<node> > nodeGroups;
        vector<int> goalGroupCount;
        vector<int> startGroupid;
        int EXPAND_RATE = 8;       // Used when generating 
        int MAX_ITER_CONNECT = 10;
};

#endif