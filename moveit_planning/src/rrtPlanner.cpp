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

    // Rviz display publisher
    markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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

    // Rviz display publisher
    markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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
            ROS_INFO("Goal node's %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }else{
        ROS_WARN("Did not find IK solution for current target pose");
    }
    
    vector<int> jointInDegree;
    node ikGoalNode;
    radianToDegree(joint_values, &jointInDegree);
    ikGoalNode.jointAngles = jointInDegree;
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
        vector<int> jointInDegree;
        radianToDegree(jointPoses, &jointInDegree);
        initialNode.jointAngles = jointInDegree;
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
    points.points.clear();
    line_list.points.clear();
}

void rrtPlanner::setVisualParam(int visualType)
{
    this->enableVisual = visualType;
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

    if(this->enableVisual != VISUAL_TYPES::NO_VISUAL){
        // Display the new node on rviz
        drawNewNode(newNode);
    }

    cout << "Current node counts is: " << rrtTree.size() << endl;
}

void rrtPlanner::drawNewNode(node newNode)
{
    // Initialize headers and other constant
    points.header.frame_id = line_list.header.frame_id = "/world";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.id = 0;
    line_list.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // Size of points and width of lines.
    points.scale.x = 0.002;
    points.scale.y = 0.002;
    line_list.scale.x = 0.001;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;   

    geometry_msgs::Point newPoint;
    geometry_msgs::Point prevPoint;
    calcNodePose(newNode, &newPoint, &prevPoint);

    points.points.push_back(newPoint);
    line_list.points.push_back(newPoint);
    line_list.points.push_back(prevPoint);

    markerPub.publish(points);
    ros::Rate sleep_time(1);
    if(this->enableVisual == VISUAL_TYPES::VISUAL_STEP){
        sleep_time.sleep();
    }    
    markerPub.publish(line_list);
}

void rrtPlanner::calcNodePose(node newNode, geometry_msgs::Point *newPose, geometry_msgs::Point *prevPose)
{
    vector<double> newJointAngles;
    degreeToRadian(newNode.jointAngles, &newJointAngles);
    kinematicState->setVariablePositions(newJointAngles);
    const Eigen::Affine3d& newEndEffectorPose = kinematicState->getGlobalLinkTransform(END_EFFECTOR_NAME);
    newPose->x = newEndEffectorPose.translation().x();
    newPose->y = newEndEffectorPose.translation().y();
    newPose->z = newEndEffectorPose.translation().z();
    cout << "newPose is: " << newPose->x << " " << newPose->y << " " << newPose->z << endl;

    vector<int> prevJointAngles = rrtTree[newNode.prevNodeid].jointAngles;
    degreeToRadian(prevJointAngles, &newJointAngles);
    kinematicState->setVariablePositions(newJointAngles);
    const Eigen::Affine3d& prevEndEffectorPose = kinematicState->getGlobalLinkTransform(END_EFFECTOR_NAME);
    prevPose->x = prevEndEffectorPose.translation().x();
    prevPose->y = prevEndEffectorPose.translation().y();
    prevPose->z = prevEndEffectorPose.translation().z();
    cout << "prevPose is: " << prevPose->x << " " << prevPose->y << " " << prevPose->z << endl;
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
    degreeToRadian(nearestNode.jointAngles, &joint_values);
    // for(size_t i=0;i<JOINTNUM;i++){
    //     joint_values.push_back(nearestNode.jointAngles[i]*DEG2PI);
    // }

    robot_state::RobotState stateInBetween(kinematicModel);
    bool isCollision = false;
    for(size_t i=0;i<FEASI_PIESCES_NUM;i++){
        for(size_t j=0;j<JOINTNUM;j++){
            joint_values[j] += (newNode.jointAngles[j] - nearestNode.jointAngles[j]) * DEG2PI / FEASI_PIESCES_NUM;
            cout << i << "th between nodes'  " << j << "th joint's angle: " << joint_values[j] << " is checking collision" << endl; 
        }
        stateInBetween.setVariablePositions(joint_names, joint_values);
        planningScene->checkCollision(collision_request, collision_result, stateInBetween);
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
    // Set some parameters i.e. this->success...
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

void rrtPlanner::degreeToRadian(vector<int> degree, vector<double> *radian)
{
    if(degree.size()!=JOINTNUM){
        ROS_ERROR("In function converting degree to radian, the size of vector does not conform with JOINTNUM");
    }else{
        if(radian->size()!=0){
            radian->clear();
        }
        for(size_t i=0;i<JOINTNUM;i++){
            radian->push_back(degree[i]*DEG2PI);
        }
    }
}

void rrtPlanner::radianToDegree(vector<double> radian, vector<int> *degree)
{
    if(radian.size()!=JOINTNUM){
        ROS_ERROR("In function converting radian to degree, the size of vector does not conform with JOINTNUM");
    }else{
        if(degree->size()!=0){
            degree->clear();
        }
        for(size_t i=0;i<JOINTNUM;i++){
            degree->push_back(radian[i]*PI2DEG);
        }
    }
}

void rrtPlanner::getParamFromCommandline(int argc, char **argv)
{
    // Get the information from command line
    ROS_INFO("Setting param as follows: ");
    int i = 1;
    while(i < argc){
        string paramName = argv[i];
        string paramValue = argv[i+1];
        setParam(paramName, paramValue);
        i += 2;
    }
}

void rrtPlanner::setParam(string paramName, string paramValue)
{
    int value = atoi(paramValue.c_str());
    if(paramName == "visual"){
        cout << "Param visual type is set to: " << paramValue;
        if(value==0 || value == 1 || value == 2){
            switch(value){
                case 0: this->enableVisual = VISUAL_TYPES::NO_VISUAL;   break;
                case 1: this->enableVisual = VISUAL_TYPES::VISUAL_ALL;   break;
                case 2: this->enableVisual = VISUAL_TYPES::VISUAL_STEP;   break;
            }
        }else{
            ROS_ERROR("The input visual type parameter is out of range");
        }
    }else if( paramName == "maxIter" ){
        if(value>0){
            this->MAXITER = value;
        }else{
            ROS_ERROR("The input maxIter is minus");
        }
    }
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