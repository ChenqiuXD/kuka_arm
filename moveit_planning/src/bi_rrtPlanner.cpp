#include "bi_rrtPlanner.h"

bi_rrtPlanner::bi_rrtPlanner(ros::NodeHandle& nh,
                             robot_model::RobotModelPtr kinematic_model,
                             planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_)
                             : rrtPlanner(nh, kinematic_model, planning_scene_monitor_)
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

bool bi_rrtPlanner::plan()
{
    // Set some parameters i.e. this->success...
    initialize();

    // The main plan process, return true if successfully planned.
    int count = 0;
    bool isGrowFrdTree = true;
    while(this->success==false && count < maxIter){
        node randNode = sampleNode();
        if(isGrowFrdTree){
            growFrdTree(randNode);
        }else{
            growBackTree(randNode);
        }
        isGrowFrdTree = !isGrowFrdTree;

        if(checkReachGoal()){
            this->success = true;
        }      

        cout << "Current node forwardTree count: " << forwardTree.size() << " backwardTree: " << backwardTree.size() << endl;
        count += 1;
    }
    if(this->success){
        findPath();
        drawPlan();
    }else{
        ROS_WARN("No available path found");
    }
    return this->success;
}

void bi_rrtPlanner::initialize()
{
    // initialize the backwardTree
    // Remember to add cluster_id to the nodes
    forwardTree.clear();
    backwardTree.clear();
    initialNode.cluster_id = 0;
    initialNode.prevNodeid = -1;
    lastFrdNode = initialNode;
    forwardTree.push_back(initialNode);
    goalNodes[0].cluster_id = 1;
    goalNodes[0].prevNodeid = -1;
    lastBackNode = goalNodes[0];
    backwardTree.push_back(goalNodes[0]);

    path.clear();
    minGoalDist = DBL_MAX;
    maxGoalDist = 0;
    success = false;

    initrrtVisual();
    initBacktreeVisual();
    initPathVisual();
}

node bi_rrtPlanner::sampleNode()
{
    // Randomly sample a node in the configuration space
    int jointAng;
    node randNode;
    randNode.prevNodeid = -1;
    srand(time(NULL));
    random_device rd;

    // if goalExtend, then 5 percent probability that the randNode is set to the goalNode
    for(int i=0; i<JOINTNUM; i++){
        jointAng = rd() % (jointUpperLimits[i]-jointLowerLimits[i]) + jointLowerLimits[i];
        randNode.jointAngles.push_back(jointAng);
    }
    
    return randNode;
}

void bi_rrtPlanner::growFrdTree(node randNode)
{
    int nearestNodeid = findNearest(randNode, forwardTree);
    node newFrdNode = extend(nearestNodeid, randNode, 0);
    if(newFrdNode.id != -1){    // if this node is feasible
        newFrdNode.cluster_id = 0;
        forwardTree.push_back(newFrdNode);
        lastFrdNode = newFrdNode;
        drawNewNode(newFrdNode);

        nearestNodeid = findNearest(newFrdNode, backwardTree);
        node newBckNode = extend(nearestNodeid, newFrdNode, 1);
        if(newBckNode.id != -1){    // if this node is feasible
            newBckNode.cluster_id = 1;
            backwardTree.push_back(newBckNode);
            lastBackNode = newBckNode;
            drawNewNode(newBckNode);
        }
    }
}

void bi_rrtPlanner::growBackTree(node randNode)
{
    int nearestNodeid = findNearest(randNode, backwardTree);
    node newBckNode = extend(nearestNodeid, randNode, 1);
    if(newBckNode.id != -1){    // if this node is feasible
        newBckNode.cluster_id = 1;
        backwardTree.push_back(newBckNode);
        lastBackNode = newBckNode;
        drawNewNode(newBckNode);

        nearestNodeid = findNearest(newBckNode, forwardTree);
        node newFrdNode = extend(nearestNodeid, newBckNode, 0);
        if(newFrdNode.id != -1){    // if this node is feasible
            newFrdNode.cluster_id = 0;
            forwardTree.push_back(newFrdNode);
            lastFrdNode = newFrdNode;
            drawNewNode(newFrdNode);
        }
    }
}

node bi_rrtPlanner::extend(int id, node randNode, int cluster_id)
{ 
    node newNode; 
    node nearestNode;
    if(cluster_id==0){nearestNode = forwardTree[id];}
    else if(cluster_id==1){nearestNode = backwardTree[id];}   

    double distance = calcDist(nearestNode, randNode);
    double step = STEP*JOINTNUM;
    if(distance<=step){
        newNode = randNode;
    }else{
        for(int i=0;i<JOINTNUM;i++){
            // The new node lies between the nearestNode and the randNode
            // with STEP*JOINTNUM distance from nearest
            int newAngle = int(nearestNode.jointAngles[i] + (randNode.jointAngles[i]-nearestNode.jointAngles[i])*step/distance);
            newNode.jointAngles.push_back(newAngle);
        }
    }

    bool isFeasible = checkFeasbility(nearestNode, newNode);
    if(isFeasible){
        newNode.prevNodeid = nearestNode.id;
        if(cluster_id==0){newNode.id = forwardTree.size();}
        else{newNode.id = backwardTree.size();}
        return newNode;
    }else{
        newNode.id = -1;
        return newNode;
    }
}

bool bi_rrtPlanner::checkReachGoal()
{
    double distance;
    bool result = true;
    for( size_t j=0;j<JOINTNUM;++j ){
        if(lastFrdNode.jointAngles.size() != JOINTNUM || lastBackNode.jointAngles.size() != JOINTNUM){
            cout << "The joint numbers which is:" << lastFrdNode.jointAngles.size() << " should be: " << JOINTNUM << endl;
            result = false;
            break;
        }
        int diff = abs(lastFrdNode.jointAngles[j] -lastBackNode.jointAngles[j]);
        // cout << "The " << j << "th joint difference is: " << diff << endl;
        if( diff > this->goalToleranceVec[j] ){
            result = false;
            break;
        }
    }
    return result;
}

void bi_rrtPlanner::findPath()
{
    node forNode, backNode;
    forNode = this->lastFrdNode;
    backNode = this->lastBackNode;
    while(backNode.prevNodeid != -1){
        this->path.push_back(backNode);
        backNode = this->backwardTree[backNode.prevNodeid];
    }
    while(forNode.prevNodeid != -1){
        this->path.push_back(forNode);
        forNode = this->forwardTree[forNode.prevNodeid];
    }
}

void bi_rrtPlanner::drawNewNode(node newNode)
{
    // Differentiate backward and forward tree
    if(newNode.cluster_id==0){
        points.header.frame_id = line_list.header.frame_id = "/world";
        points.header.stamp = line_list.header.stamp = ros::Time::now();

        geometry_msgs::Point newPoint;
        geometry_msgs::Point prevPoint;
        calcNodePose(newNode, &newPoint);
        calcNodePose(forwardTree[newNode.prevNodeid], &prevPoint);

        points.points.push_back(newPoint);
        line_list.points.push_back(newPoint);
        line_list.points.push_back(prevPoint);

        markerPub.publish(points);
        markerPub.publish(line_list); 
    }else if(newNode.cluster_id==1){
        points_bck.header.frame_id = line_list_bck.header.frame_id = "/world";
        points_bck.header.stamp = line_list_bck.header.stamp = ros::Time::now();

        geometry_msgs::Point newPoint;
        geometry_msgs::Point prevPoint;
        calcNodePose(newNode, &newPoint);
        calcNodePose(backwardTree[newNode.prevNodeid], &prevPoint);

        points_bck.points.push_back(newPoint);
        line_list_bck.points.push_back(newPoint);
        line_list_bck.points.push_back(prevPoint);

        markerPub.publish(points_bck);
        markerPub.publish(line_list_bck);
    }
    ros::WallDuration sleep_t(1);
    if(this->enableVisual == VISUAL_TYPES::VISUAL_STEP){
        sleep_t.sleep();
    }  
}

void bi_rrtPlanner::drawPlan()
{
    pathVertices.header.frame_id = pathEdges.header.frame_id = "/world";
    pathVertices.header.stamp = pathEdges.header.stamp = ros::Time::now();
    pathVertices.ns = pathEdges.ns = "path";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "tree";
    pathEdges.type = visualization_msgs::Marker::LINE_LIST;

    geometry_msgs::Point firstFrdPose, firstBackPose;
    for(size_t i = 0; i < path.size(); ++i){ 
        node curNode = path[i];
        node prevNode;
        if(curNode.prevNodeid==0 && i < path.size()-1){
            calcNodePose(path[i+1], &firstFrdPose);
            cout << "Entering next path" << endl;
        }
        if(curNode.cluster_id==0){
            prevNode = forwardTree[curNode.prevNodeid];
        }else if(curNode.cluster_id==1){
            prevNode = backwardTree[curNode.prevNodeid];
        }
        
        geometry_msgs::Point pointPose;
        geometry_msgs::Point prevPointPose;
        calcNodePose(curNode, &pointPose);
        calcNodePose(prevNode, &prevPointPose);
        pathEdges.points.push_back(pointPose);
        pathEdges.points.push_back(prevPointPose);
        pathVertices.points.push_back(pointPose);

        if(i==0){
            firstBackPose = pointPose;
        }
        cout << "Connecting " << curNode.cluster_id << " " << curNode.id << " with " << prevNode.id << " nodes. " << endl;

        if(curNode.cluster_id==0){
            points.points.erase( points.points.begin() + curNode.id - 1 );
            line_list.points.erase( line_list.points.begin() + 2*curNode.id - 1 );
            line_list.points.erase( line_list.points.begin() + 2*curNode.id - 2 );
        }else if(curNode.cluster_id==1){
            points_bck.points.erase( points_bck.points.begin() + curNode.id - 1 );
            line_list_bck.points.erase( line_list_bck.points.begin() + 2*curNode.id - 1 );
            line_list_bck.points.erase( line_list_bck.points.begin() + 2*curNode.id - 2 );
        }    
    }
    pathEdges.points.push_back(firstFrdPose);
    pathEdges.points.push_back(firstBackPose);

    ros::WallDuration sleep_t(0.1);
    markerPub.publish(pathEdges);
    sleep_t.sleep();
    markerPub.publish(pathVertices);
    sleep_t.sleep();
    markerPub.publish(line_list);
    sleep_t.sleep();
    markerPub.publish(points);
    sleep_t.sleep();
    markerPub.publish(line_list_bck);
    sleep_t.sleep();
    markerPub.publish(points_bck);
    sleep_t.sleep();
}

void bi_rrtPlanner::initBacktreeVisual()
{
    points_bck.points.clear();
    line_list_bck.points.clear();
    points.points.clear();
    line_list.points.clear();
    points_bck.ns = line_list_bck.ns = "tree";

    points_bck.action = line_list_bck.action = visualization_msgs::Marker::ADD;
    points_bck.pose.orientation.w = line_list_bck.pose.orientation.w = 1.0;
    points_bck.id = 4;
    line_list_bck.id = 5;
    points_bck.type = visualization_msgs::Marker::POINTS;
    line_list_bck.type = visualization_msgs::Marker::LINE_LIST;

    // Size of points and width of lines.
    points_bck.scale.x = 0.002;
    points_bck.scale.y = 0.002;
    line_list_bck.scale.x = 0.001;

    // Points are green
    points_bck.color.g = 1.0f;
    points_bck.color.r = 1.0f;
    points_bck.color.a = 1.0;

    // Line strip is blue
    line_list_bck.color.b = 1.0;
    line_list_bck.color.r = 1.0;
    line_list_bck.color.a = 1.0;  
}
