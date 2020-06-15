#include <ros/ros.h>
#include <string.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <vector>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <typeinfo>
#include <random>

#include "obstacle_adder.h"

using namespace std;

Obstacle_Adder::~Obstacle_Adder()
{;}

Obstacle_Adder::Obstacle_Adder(ros::NodeHandle& nh) : nh(nh)
{
    this->obs_names = {"wooden_case::base", "box::link", "kuka_arm::base_link"};;
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
                // geometry_msgs::Pose p = msg->pose[j];
            }
        }
    }
}

void Obstacle_Adder::add_obstacles()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    for(int i = 0;i < this->obs_names.size();i++){
        string element_name = obs_names[i];
        cout << element_name << endl;
        moveit_msgs::CollisionObject collision_object;
        if(element_name=="wooden_case::base"){
            collision_object = this->add_wooden_box();
        // }else if(element_name=="box::link"){ 
            // collision_object = this->add_target();  
        }else if(element_name=="kuka_arm::base_link") {
            collision_object = this->add_test_obstacles();
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

void Obstacle_Adder::generateRandomObs()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    srand(time(NULL));
    random_device rd;
    ros::WallDuration sleep_t(0.5);
    for(int i=0;i<OBJECT_NUM;++i){
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "/world";
        collision_object.header.stamp = ros::Time::now();
        collision_object.operation = collision_object.ADD;
        collision_object.id = i;
    
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        double size = rd() % (MAX_SIZE-MIN_SIZE) + MIN_SIZE;
        size /= 100;    // change from centimeter into meter
        primitive.dimensions[0] = size;
        primitive.dimensions[1] = size;
        primitive.dimensions[2] = 0.03;

        collision_object.primitives.push_back(primitive);
        geometry_msgs::Pose p = getRandomPose();
        collision_object.primitive_poses.push_back(p);

        collision_objects.push_back(collision_object);
        sleep_t.sleep();
    }

    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = "motion_planning_scene";
    planning_scene_msg.world.collision_objects = collision_objects;
    planning_scene_msg.is_diff = true;
    this->obstacle_pub.publish(planning_scene_msg);
    sleep_t.sleep();

}

void Obstacle_Adder::generateFixedObs()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/world";
    collision_object.header.stamp = ros::Time::now();
    collision_object.operation = collision_object.ADD;
    collision_object.id = "area";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = (MAX_BORDER_X - MIN_BORDER_X)/10.0;
    primitive.dimensions[1] = (MAX_BORDER_Y - MIN_BORDER_Y)/10.0;
    primitive.dimensions[2] = (MAX_BORDER_Z - MIN_BORDER_Z)/10.0;

    collision_object.primitives.push_back(primitive);
    geometry_msgs::Pose p;
    p.position.x = ((MAX_BORDER_X-MIN_BORDER_X)/2 + MIN_BORDER_X)/10.0;
    p.position.y = ((MAX_BORDER_Y-MIN_BORDER_Y)/2 + MIN_BORDER_Y)/10.0;
    p.position.z = ((MAX_BORDER_Z-MIN_BORDER_Z)/2 + MIN_BORDER_Z)/10.0;
    collision_object.primitive_poses.push_back(p);

    collision_objects.push_back(collision_object);
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.name = "motion_planning_scene";
    planning_scene_msg.world.collision_objects = collision_objects;
    planning_scene_msg.is_diff = true;
    this->obstacle_pub.publish(planning_scene_msg);  
}

moveit_msgs::CollisionObject Obstacle_Adder::add_wooden_box()
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "/world";
    collision_object.header.stamp = ros::Time::now();
    collision_object.operation = collision_object.ADD;
    string element_name = "wooden_case::base";

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

    return collision_object;
}
    
moveit_msgs::CollisionObject Obstacle_Adder::add_target()
{
    string element_name = "box::link";
    moveit_msgs::CollisionObject collision_object;
    ROS_INFO("Detected unit_box");
    collision_object.id = "box::link";
    collision_object.header.frame_id = "/world";
    collision_object.header.stamp = ros::Time::now();
    collision_object.operation = collision_object.ADD;
    
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.08;
    primitive.dimensions[1] = 0.08;
    primitive.dimensions[2] = 0.08;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(this->obs_poses[element_name]);

    return collision_object;
}

moveit_msgs::CollisionObject Obstacle_Adder::add_test_obstacles()
{
    moveit_msgs::CollisionObject collision_object;
    ROS_INFO("Adding test box");
    collision_object.id = "test_box";
    collision_object.header.frame_id = "/world";
    collision_object.header.stamp = ros::Time::now();
    collision_object.operation = collision_object.ADD;
    
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.10;
    primitive.dimensions[1] = 0.10;
    primitive.dimensions[2] = 0.03;
    collision_object.primitives.push_back(primitive);

    primitive.dimensions[0] = 0.10;
    primitive.dimensions[1] = 0.10;
    primitive.dimensions[2] = 0.03;
    collision_object.primitives.push_back(primitive);
    
    // primitive.dimensions[0] = 0.06;
    // primitive.dimensions[1] = 0.06;
    // primitive.dimensions[2] = 0.03;
    // collision_object.primitives.push_back(primitive);

    geometry_msgs::Pose pose;
    pose.position.x = 0.60;
    pose.position.y = -0.04;
    pose.position.z = 0.7;
    collision_object.primitive_poses.push_back(pose);

    pose.position.x = 0.60;
    pose.position.y = 0.2;
    pose.position.z = 0.6;
    collision_object.primitive_poses.push_back(pose);

    // pose.position.x = 0.45;
    // pose.position.y = 0.1;
    // pose.position.z = 0.5;
    // collision_object.primitive_poses.push_back(pose);

    return collision_object;
}

geometry_msgs::Pose Obstacle_Adder::getRandomPose()
{
    srand(time(NULL));
    random_device rd;
    geometry_msgs::Pose p;
    p.position.x = rd() % (MAX_BORDER_X-MIN_BORDER_X);
    p.position.y = rd() % (MAX_BORDER_Y-MIN_BORDER_Y);
    p.position.z = rd() % (MAX_BORDER_Z-MIN_BORDER_Z);
    p.position.x += MIN_BORDER_X;
    p.position.y += MIN_BORDER_Y;
    p.position.z += MIN_BORDER_Z;
    p.position.x /= 10;
    p.position.y /= 10;
    p.position.z /= 10;
    return p;
}