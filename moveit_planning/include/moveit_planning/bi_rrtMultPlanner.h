#ifndef BI_RRT_MULT_PLANNER_H
#define BI_RRT_MULT_PLANNER_H

#include "bi_rrtPlanner.h"

class bi_rrtMultPlanner : public bi_rrtPlanner
{
    public:
        bi_rrtMultPlanner(ros::NodeHandle& nh,
                          robot_model::RobotModelPtr kinematic_model,
                          planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
        
        // core function
        bool plan();
        void getSimplePath();
        void seperateSimplePaths();
        void seperatePath(vector<node> &path);
        void addToTree();

        // util function
        double getDistStartToEnd();
        // void findPath();
        // void drawPlan();

        // members
        int groupCount;
        vector< vector<node> > simplePaths;
        vector< vector<node> > nodeGroups;
        vector<int> goalGroupCount;
        vector<int> startGroupid;
        int EXPAND_RATE = 8;       // Used when generating 
};

#endif