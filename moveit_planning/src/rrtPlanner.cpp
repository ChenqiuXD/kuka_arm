#include "rrtPlanner.h"
#include <sstream>
#include <math.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace std;

rrtPlanner::rrtPlanner(ros::NodeHandle& nh, geometry_msgs::Pose objPose) : nh(nh)
{
    generateGraspPose(objPose);
    
    // Get the joint limits from ros server
    int upper, lower;
    stringstream ss;
    ROS_INFO("From rosparam getting joints' upper and lower limits");
    for(u_int i=0; i<JOINTNUM; i++){
        ss << "/joint_limits/joint_a" << i+1 << "/upper";
        nh.getParam(ss.str(),upper);
        ss.str("");
        jointUpperLimits[i] = upper;
        // cout << "The upper is: " << jointUpperLimits[i] << endl;

        ss << "/joint_limits/joint_a" << i+1 << "/lower";
        nh.getParam(ss.str(),lower);
        ss.str("");
        jointLowerLimits[i] = lower;
        // cout << "The lower is: " << jointLowerLimits[i] << endl;
    }
}

void rrtPlanner::generateGraspPose(geometry_msgs::Pose objPose)
{
    // From objPose get the target end-effector pose.
    ; 
}

node rrtPlanner::sampleNode()
{
    // Randomly sample a node in the configuration space
    int jointAng;
    node randNode;
    randNode.prevNodeid = -1;
    srand(time(NULL));
    for(int i=0; i<JOINTNUM; i++){
        jointAng = rand() % (jointUpperLimits[i]-jointLowerLimits[i]) + jointLowerLimits[i];
        randNode.jointAngles[i] = jointAng;
        cout << "The " << i << "th joint's angle is: " << randNode.jointAngles[i] << endl;
    }

    return randNode;
}

int rrtPlanner::findNearest(node randNode)
{
    // Use brute-force method to find the nearest node among the tree
    // return the id of that nearest node
    double dist, minDist=float('inf');
    int minid = -1;
    for(int i=0;i<rrtTree.size();i++){
        dist = calcDist(rrtTree[i], randNode); 
        if(dist<minDist){
            minDist = dist;
            minid = i;
        }
    }
    cout << "The minid is: " << minid << " and the min dist is: " << minDist <<endl;
    return minid;
}

double rrtPlanner::calcDist(node a, node b)
{
    double result, distance = 0;
    double angleDist = 0;
    for(int i=0;i<JOINTNUM;i++){
        // NOTE that here you can divide the minus with the (UpperLimit - LowerLimit)
        angleDist = (a.jointAngles[i] - b.jointAngles[i]) / (1);
        distance += pow(angleDist,2);
    }
    result = sqrt(distance);
    return result;
}

void rrtPlanner::extend(int id, node randNode)
{
    // Check the feasibility and extend the node
    node nearestNode = rrtTree[id];
    double distance = calcDist(nearestNode, randNode);

    if(distance<=STEP){
        node newNode = randNode;
    }else{
        node newNode;
        for(int i=0;i<JOINTNUM;i++){
            newNode.jointAngles[i] = nearestNode.jointAngles[i] + this->STEP * (randNode.jointAngles[i]-nearestNode.jointAngles[i]) /distance;
        }
        bool isReachGoal = checkReachGoal(newNode);
        if(isReachGoal){
            this->success = true;
        }else{
            rrtTree.push_back(newNode);
        }
    }
}

bool rrtPlanner::checkReachGoal(node newNode)
{
    // Use kinematics to calculate the distance between newNode's end-effector's pose and the goalPose
    double distance;
    bool result = false;
    for(int i=0;i<goalNodes.size();i++){
        distance = calcDist(newNode, goalNodes[i]);
        if(distance<=GOALTOLERANCE){
            result = true;
            break;
        }
    }
    return result;
}

bool rrtPlanner::checkFeasbility(node nearestNode)
{
    // Check the straight line between the nearest node and the new node.
    ;
}


void rrtPlanner::findPath()
{
    ;
}

bool rrtPlanner::plan()
{
    // The main plan process, return true if successfully planned.
    node initialNode;
    initialNode.id = 0;
    for(int i=0;i<JOINTNUM;i++){initialNode.jointAngles[i]=0;}
    initialNode.prevNodeid = -1;
    this->rrtTree.push_back(initialNode);

    node randNode = sampleNode();
    int nearestNodeid = findNearest(randNode);
    extend(nearestNodeid, randNode);

    return this->success;
}

int main(int argc, char** argv)
{
    // For test
    ros::init(argc, argv, "pos_listener");
    ros::NodeHandle nh;
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    geometry_msgs::Pose pose;
    pose.position.z = -1;
    pose.orientation.w = -1;

    rrtPlanner planner = rrtPlanner(nh, pose);
    bool success = planner.plan();
    ROS_INFO_NAMED("rrtPlanner", " planned %s", success ? "SUCCESFULLY" : "FAILED");
}