#include <ros/ros.h>
#include <string.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <vector>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <typeinfo>

#include "obstacle_adder.h"

using namespace std;

Obstacle_Adder::~Obstacle_Adder()
{;}

Obstacle_Adder::Obstacle_Adder(ros::NodeHandle& nh) : nh(nh)
{
    this->obs_names = {"wooden_case::base", "box::link"};;
    this->link_states_sub = nh.subscribe("/gazebo/link_states", 1, &Obstacle_Adder::obs_callback, this);
    this->obstacle_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
}

void Obstacle_Adder::obs_callback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    string element_name;
    int obs_length = this->obs_names.size();
    int msg_length = msg->name.size();

    // Traverse the link_states->name to get the index
    for(int i=0;i<obs_length;i++){
        element_name = this->obs_names[i];
        for(int j=0;j<msg_length;j++){
            if(msg->name[j]==element_name){
                this->obs_poses[element_name] = msg->pose[j];
                geometry_msgs::Pose p = msg->pose[j];
            }
        }
    }
}

void Obstacle_Adder::add_obstacles()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    for(int i = 0;i < this->obs_names.size();i++){
        string element_name = obs_names[i];
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "/world";
        collision_object.header.stamp = ros::Time::now();
        if(element_name=="wooden_case::base"){
            ROS_INFO("Detected wooden_box");
            collision_object.id = "wooden_case::base";
            
            // Add the box front
            shape_msgs::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = 0.16;
            primitive.dimensions[1] = 0.56;
            primitive.dimensions[2] = 0.02;
            collision_object.primitives.push_back(primitive);
            geometry_msgs::Pose p;
            p = this->obs_poses[element_name];
            collision_object.primitive_poses.push_back(p);

            // Add the box back : The size is the same, only need to change the pose
            collision_object.primitives.push_back(primitive);
            p = this->obs_poses[element_name];
            p.position.x -= 0.58; // The distance between wooden_case front and back
            collision_object.primitive_poses.push_back(p);

            // Add the left part            
            primitive.dimensions[0] = 0.16;
            primitive.dimensions[1] = 0.02;
            primitive.dimensions[2] = 0.60;
            collision_object.primitives.push_back(primitive);
            p = this->obs_poses[element_name];
            p.position.y -= 0.29; // The distance between wooden_case front and left
            p.position.x -= 0.29;
            collision_object.primitive_poses.push_back(p);

            // Add the right part
            collision_object.primitives.push_back(primitive);
            p = this->obs_poses[element_name];
            p.position.y += 0.29; // The distance between wooden_case front and left
            p.position.x -= 0.29;
            collision_object.primitive_poses.push_back(p); 

            // Add the bottom part        
            primitive.dimensions[0] = 0.02;
            primitive.dimensions[1] = 0.56;
            primitive.dimensions[2] = 0.56;
            collision_object.primitives.push_back(primitive);
            p = this->obs_poses[element_name];
            p.position.x -= 0.30; // The distance between wooden_case front and left
            p.position.z -= 0.07;
            collision_object.primitive_poses.push_back(p);

            collision_object.operation = collision_object.ADD;
        // }else if(element_name=="box::link"){            
        //     ROS_INFO("Detected unit_box");
        //     collision_object.id = "box::link";
            
        //     shape_msgs::SolidPrimitive primitive;
        //     primitive.type = primitive.BOX;
        //     primitive.dimensions.resize(3);
        //     primitive.dimensions[0] = 0.08;
        //     primitive.dimensions[1] = 0.08;
        //     primitive.dimensions[2] = 0.08;

        //     collision_object.primitives.push_back(primitive);
        //     collision_object.primitive_poses.push_back(this->obs_poses[element_name]);
        //     collision_object.operation = collision_object.ADD;
        }else{
            continue;
        }
        collision_objects.push_back(collision_object);
    }
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = "motion_planning_scene";
    planning_scene_msg.world.collision_objects = collision_objects;
    planning_scene_msg.is_diff = true;
    this->obstacle_pub.publish(planning_scene_msg);
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
}
