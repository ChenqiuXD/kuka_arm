#ifndef RRT_CONNECT_PLANNER_H
#define RRT_CONNECT_PLANNER_H

#include "bi_rrtPlanner.h"

class rrtConnectPlanner : public bi_rrtPlanner
{
    public:
        rrtConnectPlanner(ros::NodeHandle& nh,
                      robot_model::RobotModelPtr kinematic_model,
                      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
        
        // Core functions
        bool plan();
        void growFrdTree(node randNode);
        void growBackTree(node randNode);
        void connect(int, node, int);
        vector<int> newJoints(node, node);
        // node extend(int id, node randNode, int cluster_id);

        // util functions
        void initialize();

        // Member
        visualization_msgs::Marker points_bck;
        visualization_msgs::Marker line_list_bck;

        int MAX_ITER_CONNECT = 10;
};

#endif