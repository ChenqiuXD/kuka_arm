#ifndef BI_RRT_PLANNGER_H
#define BI_RRT_PLANNGER_H

#include "rrtPlanner.h"

class bi_rrtPlanner : public rrtPlanner
{
    public:
        // bi_rrtPlanner(){;}
        bi_rrtPlanner(ros::NodeHandle& nh,
                      robot_model::RobotModelPtr kinematic_model,
                      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_);
        
        // bi_RRT core functions
        void growFrdTree(node);
        void growBackTree(node);
        node extend(int id, node randNode, int type);   // type: 0 -> forward, 1 -> backward
        bool checkReachGoal();
        bool plan();
        void findPath();

        // bi_RRT util functions
        void initialize();
        void drawPlan();
        void initBacktreeVisual();
        void drawNewNode(node);

        vector<node> forwrdTree;
        vector<node> backwrdTree;
        node lastFrdNode;
        node lastBackNode;
        visualization_msgs::Marker points_bck;
        visualization_msgs::Marker line_list_bck;
};

#endif