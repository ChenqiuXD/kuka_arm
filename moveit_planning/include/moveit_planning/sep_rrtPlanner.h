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
        void seperateSimplePath();
        void checkConnection(node newNode, int *groupid, int *nodeid);
        void connectToGroup(node *newNode, int groupid, int nodeid);

        // utils
        double getDistStartToEnd();
        void drawSimplePath();
        void initVisual();

        int groupCount;
        vector<node> simplePath;
        vector< vector<node> > nodeGroups;
        vector<node> path;

        visualization_msgs::Marker points_simple;
        visualization_msgs::Marker line_list_simple;
};

#endif