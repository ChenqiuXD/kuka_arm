#include "rrtMultConnect.h"

rrtMultConnectPlanner::rrtMultConnectPlanner(ros::NodeHandle& nh,
                                             robot_model::RobotModelPtr kinematic_model,
                                             planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_)
                                            : rrtConnectPlanner(nh, kinematic_model, planning_scene_monitor_)
{
    this->kinematicModel = kinematic_model;
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    this->kinematicState = kinematic_state;
    this->kinematicState->setToDefaultValues();
    this->jointModelGroup = kinematic_model->getJointModelGroup("manipulator");
    this->planningSceneMonitor_ = planning_scene_monitor_;

    // Rviz display publisher
    // this->visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("/world","/visualization_marker"));
    this->markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // Readin joint limits
    readJntLimits(); 
}

bool rrtMultConnectPlanner::planPath()
{
    ;
}