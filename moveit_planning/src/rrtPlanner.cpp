#include "rrtPlanner.h"
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sstream>
#include <math.h>
#include <random>

using namespace std;

rrtPlanner::rrtPlanner(ros::NodeHandle& nh,
                       geometry_msgs::Pose objPose,
                       robot_model::RobotModelPtr kinematic_model,
                       planning_scene::PlanningScene* planning_scene) : nh(nh)
{
    this->kinematicModel = kinematic_model;
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    this->kinematicState = kinematic_state;
    this->kinematicState->setToDefaultValues();
    this->jointModelGroup = kinematic_model->getJointModelGroup("manipulator");
    this->planningScene = planning_scene;

    // Get the target end-effector poses
    generateGraspPose(objPose);

    // Get the joint limits from ros server
    int upper, lower;
    stringstream ss;
    for(size_t i=0; i<JOINTNUM; i++){
        ss << "/joint_limits/joint_a" << i+1 << "/upper";
        bool isLimitsFound = nh.getParam(ss.str(),upper); 
        if(!isLimitsFound){
            ROS_ERROR("No limits information found, check whether rrtPlanner.launch is launched");
            break;
        }
        ss.str("");
        jointUpperLimits[i] = upper;
        cout << "The upper is: " << jointUpperLimits[i] << endl;

        ss << "/joint_limits/joint_a" << i+1 << "/lower";
        nh.getParam(ss.str(),lower);
        ss.str("");
        jointLowerLimits[i] = lower;
        cout << "The lower is: " << jointLowerLimits[i] << endl;
    }
}

rrtPlanner::rrtPlanner(ros::NodeHandle& nh,
            robot_model::RobotModelPtr kinematic_model,
            planning_scene::PlanningScene* planning_scene) : nh(nh)
{
    this->kinematicModel = kinematic_model;
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    this->kinematicState = kinematic_state;
    this->kinematicState->setToDefaultValues();
    this->jointModelGroup = kinematic_model->getJointModelGroup("manipulator");
    this->planningScene = planning_scene;

    // Get the joint limits from ros server
    int upper, lower;
    stringstream ss;
    for(size_t i=0; i<JOINTNUM; i++){
        ss << "/joint_limits/joint_a" << i+1 << "/upper";
        bool isLimitsFound = nh.getParam(ss.str(),upper); 
        if(!isLimitsFound){
            ROS_ERROR("No limits information found, check whether rrtPlanner.launch is launched");
            break;
        }
        ss.str("");
        jointUpperLimits[i] = upper;
        cout << "The upper is: " << jointUpperLimits[i] << endl;

        ss << "/joint_limits/joint_a" << i+1 << "/lower";
        nh.getParam(ss.str(),lower);
        ss.str("");
        jointLowerLimits[i] = lower;
        cout << "The lower is: " << jointLowerLimits[i] << endl;
    }           
}

void rrtPlanner::generateGraspPose(geometry_msgs::Pose objPose)
{
    const std::vector<std::string>& joint_names = jointModelGroup->getVariableNames();
    std::vector<double> joint_values;
    kinematicState->copyJointGroupPositions(jointModelGroup, joint_values);

    Eigen::Vector3d trans;
    trans << objPose.position.x, objPose.position.y, objPose.position.z;
    Eigen::Quaterniond quater(objPose.orientation.w, objPose.orientation.x, objPose.orientation.y, objPose.orientation.z); 
    Eigen::Affine3d end_effector_state = Eigen::Affine3d::Identity();
    end_effector_state.translation() = trans;
    end_effector_state.linear() = quater.toRotationMatrix();

    // cout << end_effector_state.translation( ) << endl;
    // cout << end_effector_state.rotation() << endl;

    std::size_t attempts = 10;
    double timeout = 0.1;
    bool found_ik = kinematicState->setFromIK(jointModelGroup, end_effector_state, attempts, timeout);
    if (found_ik){
        kinematicState->copyJointGroupPositions(jointModelGroup, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }else{
        ROS_WARN("Did not find IK solution for current target pose");
    }
    
    node ikGoalNode;
    for(size_t i=0;i<JOINTNUM;i++){
        ikGoalNode.jointAngles.push_back((int)(joint_values[i] * PI2DEG));
        cout << "Goal node" << i <<"th joint is:" << ikGoalNode.jointAngles[i] << endl;
    }
    goalNodes.push_back(ikGoalNode);
}

void rrtPlanner::setInitialNode(vector<int> jointPoses)
{
    if(jointPoses.size() != JOINTNUM){
        ROS_ERROR("Could not set start node, the joint count is not correct");
        cout << "Should be: " << JOINTNUM << ". But the input num is: " << jointPoses.size() << endl;
    }else{
        initialNode.id = 0;
        initialNode.jointAngles = jointPoses;
    }
}

void rrtPlanner::setInitialNode(vector<double> jointPoses)
{
    if(jointPoses.size() != JOINTNUM){
        ROS_ERROR("Could not set start node, the joint count is not correct");
        cout << "Should be: " << JOINTNUM << ". But the input num is: " << jointPoses.size() << endl;
    }else{
        initialNode.id = 0;
        for(size_t i=0;i<6;i++){
            initialNode.jointAngles.push_back(jointPoses[i]*PI2DEG);
            cout << "The " << i << "th joint value is: " << initialNode.jointAngles[i] << endl;
        }
        ROS_INFO("Initial node set");
        cout << "Current node size is: " << rrtTree.size() << endl;
    }
}

void rrtPlanner::initialize()
{
    if(rrtTree.size()!=0){
        rrtTree.clear();
    }
    rrtTree.push_back(initialNode);
    minGoalDist = float('inf');
    maxGoalDist = 0;
    success = false;
}

node rrtPlanner::sampleNode()
{
    // Randomly sample a node in the configuration space
    int jointAng;
    node randNode;
    randNode.prevNodeid = -1;
    srand(time(NULL));
    random_device rd;
    for(int i=0; i<JOINTNUM; i++){
        jointAng = rd() % (jointUpperLimits[i]-jointLowerLimits[i]) + jointLowerLimits[i];
        randNode.jointAngles.push_back(jointAng);
        cout << "The " << i << "th joint's sampled node is: " << randNode.jointAngles[i] << endl;
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
    ROS_INFO("Entering extend function");
    // Check the feasibility and extend the node
    if(id==-1){
        ROS_ERROR("Nearest node's id is -1, check the size of rrtTree: %d", (int)rrtTree.size());
        return;
    }
    node nearestNode = rrtTree[id];
    double distance = calcDist(nearestNode, randNode);
    node newNode;

    double step = STEP*JOINTNUM;
    if(distance<=step){
        newNode = randNode;
    }else{
        for(int i=0;i<JOINTNUM;i++){
            // The new node lies between the nearestNode and the randNode
            // with STEP*JOINTNUM distance from nearest
            int newAngle = int(nearestNode.jointAngles[i] + (randNode.jointAngles[i]-nearestNode.jointAngles[i])*step/distance);
            newNode.jointAngles.push_back(newAngle);
            cout << "The new angle is: " << newAngle << endl;
        }
    }

    for(size_t i=0;i<JOINTNUM;i++){
        cout << "Nearest Node " << i << "th angle is: " <<nearestNode.jointAngles[i] << endl;
    }

    bool isFeasible = checkFeasbility(nearestNode, newNode);
    if(isFeasible){
        bool isReachGoal = checkReachGoal(newNode);
        newNode.prevNodeid = nearestNode.id;
        newNode.id = rrtTree.size();
        rrtTree.push_back(newNode);
        if(isReachGoal){
            this->success = true;
        }
    }
    cout << "Current node counts is: " << rrtTree.size() << endl;
}

bool rrtPlanner::checkReachGoal(node newNode)
{
    // Use kinematics to calculate the distance between newNode's end-effector's pose and the goalPose
    double distance;
    bool result = false;
    for(int i=0;i<goalNodes.size();i++){
        distance = calcDist(newNode, goalNodes[i]);
        if(distance < minGoalDist){minGoalDist = distance;}
        if(distance > maxGoalDist){maxGoalDist = distance;}
        cout << "New node distance to goal is: " << distance << endl;
        if(distance<=GOALTOLERANCE*JOINTNUM){
            result = true;
            break;
        }
    }
    return result;
}

bool rrtPlanner::checkFeasbility(node nearestNode, node newNode)
{
    ROS_INFO("Entering checkFeasibility function");
    // Check the straight line between the nearest node and the new node.
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    const std::vector<std::string>& joint_names = jointModelGroup->getVariableNames();
    std::vector<double> joint_values;
    for(size_t i=0;i<JOINTNUM;i++){
        joint_values.push_back(nearestNode.jointAngles[i]*DEG2PI);
    }

    robot_state::RobotState stateBetween(kinematicModel);
    bool isCollision = false;
    for(size_t i=0;i<FEASI_PIESCES_NUM;i++){
        for(size_t j=0;j<JOINTNUM;j++){
            joint_values[j] += (newNode.jointAngles[j] - nearestNode.jointAngles[j]) * DEG2PI / FEASI_PIESCES_NUM;
            cout << i << "th between nodes'  " << j << "th joint's angle: " << joint_values[j] << " is checking collision" << endl; 
        }
        stateBetween.setVariablePositions(joint_names, joint_values);
        planningScene->checkCollision(collision_request, collision_result, stateBetween);
        if(collision_result.collision){
            isCollision = true;
            cout << i << "th test encounters a collision, node abandoned" << endl;
            break;
        }
    }
    return !isCollision;
}

void rrtPlanner::findPath()
{
    cout << "Starting from the last node: " << rrtTree.size()-1 << endl;
    node curNode = rrtTree.back();
    while(curNode.id!=0){
        this->path.push_back(curNode);
        cout << "Connecting " << curNode.id << "th node with " << curNode.prevNodeid << "th node. " << endl;
        curNode = rrtTree[curNode.prevNodeid];
    }
}

void rrtPlanner::generatePlan(double time, moveit::planning_interface::MoveGroupInterface::Plan* my_plan)
{
    my_plan->planning_time_ = time;
    
    moveit_msgs::RobotState robot_state;

    sensor_msgs::JointState joint_state;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "/world";
    joint_state.header = header;

    vector<string> s;
    nh.getParam("/joint_names", s);
    joint_state.name = s;

    // TODO
}

bool rrtPlanner::plan()
{
    // Set some parameters i.e. this->success
    initialize();

    // The main plan process, return true if successfully planned.
    int count = 0;
    while(this->success==false && count < MAXITER){
        node randNode = sampleNode();
        int nearestNodeid = findNearest(randNode);
        extend(nearestNodeid, randNode);
        count += 1;
    }
    if(this->success){
        findPath();
    }else{
        ROS_WARN("No available path found");
    }
    cout << "Minimum distance is: " << minGoalDist << endl;
    cout << "Maximum distance is: " << maxGoalDist << endl;
    return this->success;
}

// int main(int argc, char** argv)
// {
//     // For test
//     ros::init(argc, argv, "pos_listener");
//     ros::NodeHandle nh;
    
//     robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//     robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//     planning_scene::PlanningScene planning_scene(kinematic_model);
//     ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

//     geometry_msgs::Pose pose;
//     pose.orientation.x = 0.69867;
//     pose.orientation.y = 0.70328;
//     pose.orientation.z = 0.0860347;
//     pose.orientation.w = 0.0992733;
//     pose.position.x = 0.608802;
//     pose.position.y = 0.105058;
//     pose.position.z = 0.408134;

//     rrtPlanner planner = rrtPlanner(nh, pose, kinematic_model, &planning_scene);
//     bool success = planner.plan();
//     ROS_INFO_NAMED("rrtPlanner", " planned %s", success ? "SUCCESFULLY" : "FAILED");
// }