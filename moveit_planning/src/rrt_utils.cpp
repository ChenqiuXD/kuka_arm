#include "rrtPlanner.h"
#include <iostream>

void rrtPlanner::readJntLimits()
{
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

void rrtPlanner::setGoalNode(vector<int> jointPosition)
{
    node ikGoalNode;
    ikGoalNode.id = 0;
    ikGoalNode.jointAngles = jointPosition;
    goalNodes.push_back(ikGoalNode);
}

void rrtPlanner::setGoalNode(vector<double> jointPosition)
{
    vector<int> jointInDegree;
    radianToDegree(jointPosition, &jointInDegree);
    setGoalNode(jointInDegree);
}

void rrtPlanner::setGoalNodeFromPose(geometry_msgs::Pose objPose)
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
    
    setGoalNode(joint_values);
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
    vector<int> jointInDegree;
    radianToDegree(jointPoses, &jointInDegree);
    setInitialNode(jointInDegree);
}

void rrtPlanner::initialize()
{
    if(rrtTree.size()!=0){
        rrtTree.clear();
    }
    path.clear();
    minGoalDist = DBL_MAX;
    maxGoalDist = 0;
    success = false;

    // Tried, but useless
    // this->visual_tools_->deleteAllMarkers(); 
    initrrtVisual();
    initPathVisual();
}

void rrtPlanner::initrrtVisual()
{
    points.points.clear();
    line_list.points.clear();

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
}

void rrtPlanner::initPathVisual()
{
    pathVertices.points.clear();
    pathEdges.points.clear();
    pathVertices.header.frame_id = pathEdges.header.frame_id = "/world";
    pathVertices.header.stamp = pathEdges.header.stamp = ros::Time::now();

    markerPub.publish(pathVertices);
    markerPub.publish(pathEdges);

    pathVertices.action = pathEdges.action = visualization_msgs::Marker::ADD;
    pathVertices.pose.orientation.w = pathEdges.pose.orientation.w = 1.0;
    pathVertices.id = 2;
    pathEdges.id = 3;
    pathVertices.type = visualization_msgs::Marker::POINTS;
    pathEdges.type = visualization_msgs::Marker::LINE_STRIP;

    // Size of points and width of lines.
    pathVertices.scale.x = 0.002;
    pathVertices.scale.y = 0.002;
    pathEdges.scale.x = 0.001;

    // Points are green
    pathVertices.color.g = 1.0f;
    pathVertices.color.b = 1.0f;
    pathVertices.color.a = 1.0;

    // Line strip is blue
    pathEdges.color.r = 1.0;
    pathEdges.color.a = 1.0; 
}

void rrtPlanner::drawNewNode(node newNode)
{
    // Initialize headers and other constant
    points.header.frame_id = line_list.header.frame_id = "/world";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "tree";

    geometry_msgs::Point newPoint;
    geometry_msgs::Point prevPoint;
    calcNodePose(newNode, &newPoint);
    calcNodePose(rrtTree[newNode.prevNodeid], &prevPoint);

    points.points.push_back(newPoint);
    line_list.points.push_back(newPoint);
    line_list.points.push_back(prevPoint);

    markerPub.publish(points);
    markerPub.publish(line_list);
    ros::WallDuration sleep_t(1);
    if(this->enableVisual == VISUAL_TYPES::VISUAL_STEP){
        sleep_t.sleep();
    }   
}

void rrtPlanner::drawPlan()
{
    pathVertices.header.frame_id = pathEdges.header.frame_id = "/world";
    pathVertices.header.stamp = pathEdges.header.stamp = ros::Time::now();
    pathVertices.ns = pathEdges.ns = "path";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "tree";

    for(size_t i = 0; i < path.size()-1; ++i){  // -1 is to exempt the initialNode
        node curNode = path[i];
        geometry_msgs::Point pointPose;
        calcNodePose(curNode, &pointPose);
        pathEdges.points.push_back(pointPose);
        pathVertices.points.push_back(pointPose);

        points.points.erase( points.points.begin() + curNode.id - 1 );
        line_list.points.erase( line_list.points.begin() + 2*curNode.id - 1 );
        line_list.points.erase( line_list.points.begin() + 2*curNode.id - 2 );
    }
    ros::WallDuration sleep_t(0.1);
    markerPub.publish(pathEdges);
    sleep_t.sleep();
    markerPub.publish(pathVertices);
    sleep_t.sleep();
    markerPub.publish(line_list);
    sleep_t.sleep();
    markerPub.publish(points);
    sleep_t.sleep();
}

void rrtPlanner::calcNodePose(node newNode, geometry_msgs::Point *nodePose)
{
    vector<double> newJointAngles;
    degreeToRadian(newNode.jointAngles, &newJointAngles);
    kinematicState->setVariablePositions(newJointAngles);
    const Eigen::Affine3d& newEndEffectorPose = kinematicState->getGlobalLinkTransform(END_EFFECTOR_NAME);
    nodePose->x = newEndEffectorPose.translation().x();
    nodePose->y = newEndEffectorPose.translation().y();
    nodePose->z = newEndEffectorPose.translation().z();
}

void rrtPlanner::generatePlanMsg(double time, moveit::planning_interface::MoveGroupInterface::Plan* my_plan)
{
    // my_plan->planning_time_ = time;    
    // moveit_msgs::RobotState robot_state;
    // sensor_msgs::JointState joint_state;

    // std_msgs::Header header;
    // header.stamp = ros::Time::now();
    // header.frame_id = "/world";
    // joint_state.header = header;

    // vector<string> s;
    // nh.getParam("/joint_names", s);
    // joint_state.name = s;

    // TODO
    ;
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
            this->maxIter = value;
        }else{
            ROS_ERROR("The input maxIter is minus");
        }
    // }else if( paramName == "tolerance" ){
    //     if(value>0){
    //         this->goalTolerance = value;
    //     }else{
    //         ROS_ERROR("The tolerance must bigger than 0");
    //     }
    }else if( paramName == "goalExtend" ){
        if(value==0 || value==1){
            this->goalExtend = value;    
        }else{
            ROS_ERROR("The goalExtend only accept 1 or 0");
        }
        
    }
}

void rrtPlanner::setVisualParam(int visualType)
{
    this->enableVisual = visualType;
}

void rrtPlanner::vecInt2Double(vector<int> a, vector<double> *b)
{
    if(b->size()==a.size()){
        for(size_t i=0;i<a.size();++i){
            (*b)[i] = a[i];
        }
    }else if(b->size()==0){
        for(size_t i=0;i<a.size();++i){
            b->push_back(a[i]);
        }
    }
}

void rrtPlanner::vecDoub2Int(vector<double> a, vector<int> *b)
{
    if(b->size()==a.size()){
        for(size_t i=0;i<a.size();++i){
            (*b)[i] = a[i];
        }
    }else if(b->size()==0){
        for(size_t i=0;i<a.size();++i){
            b->push_back(a[i]);
        }
    }
}