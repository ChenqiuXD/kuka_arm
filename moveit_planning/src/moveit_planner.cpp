#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


class moveitPlanner
{
    public:
        geometry_msgs::Pose targetPos;
    
    public:
        moveitPlanner();
        ~moveitPlanner();
        void printTargetPose();
        void plan();
        void posCallBack(const geometry_msgs::Pose::ConstPtr& msg);

    private:
        moveit::planning_interface::MoveGroupInterface moveGroup("manipulator");
        moveit::planning_interface::PlanningSceneInterface planningScene;
        robot_state::JointModelGroup* joint_model_group;
};

moveitPlanner::moveitPlanner()
{
    moveGroup = new moveit::planning_interface::MoveGroupInterface("manipulator");
    planningScene = new moveit::planning_interface::PlanningSceneInterface();
    joint_model_group = moveGroup.getCurrentState()->getJointModelGroup("manipulator");
}

moveitPlanner::~moveitPlanner()
{
    delete moveGroup;
    delete planningScene;
}

void moveitPlanner::printTargetPose()
{
    std::string s_x = std::to_string(targetPos.position.x);
    std::string s_w = std::to_string(targetPos.orientation.w);
    std::cout << "The x position is " << s_x << " and the w orientation is " << s_w << std::endl;
}

void moveitPlanner::posCallBack(const geometry_msgs::Pose::ConstPtr& msg)
{
    // ROS_INFO("Heard hello parameters");
    targetPos.position = msg->position;
    targetPos.orientation = msg->orientation;
}

void moveitPlanner::plan()
{
    //TODO
    std::cout << "Planning Path towards the target Pose" << std::endl ;
    printTargetPose();
}

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "pos_listener");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    moveitPlanner p;
    ros::Subscriber sub = nh.subscribe("/kuka_arm/target_pos", 1, &moveitPlanner::posCallBack, &p);
    
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl(); 

    while(ros::ok()){
        visual_tools.prompt("Press 'next' to plan a path");

        ros::spinOnce();
        p.plan();
        loop_rate.sleep();
    }

    return 0;
}