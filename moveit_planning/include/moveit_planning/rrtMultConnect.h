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
        bool planPath();

        // util functions

        // Member
        int MAX_ITER_CONNECT = 10;
};

#endif