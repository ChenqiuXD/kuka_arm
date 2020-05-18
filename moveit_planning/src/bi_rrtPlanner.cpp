#include "bi_rrtPlanner.h"

bi_rrtPlanner::bi_rrtPlanner(ros::NodeHandle& nh,
                             robot_model::RobotModelPtr kinematic_model,
                             planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_)
                             : rrtPlanner(nh, kinematic_model, planning_scene_monitor_)
{
    // rrtPlanner(nh, kinematic_model, planning_scene_monitor_);
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

node bi_rrtPlanner::extend(int id, node randNode, int type)
{ 
    node newNode;    
    // Check the feasibility and extend the node
    if(id==-1){
        ROS_ERROR("Nearest node's id is -1, check the size of tree");
        newNode.id = -1;
        return newNode;
    }

    node nearestNode;
    if(type==0){nearestNode = forwrdTree[id];}
    else if(type==1){nearestNode = backwrdTree[id];}   

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
        if(type==0){newNode.id = forwrdTree.size();}
        else{newNode.id = backwrdTree.size();}
        return newNode;
    }else{
        newNode.id = -1;
        return newNode;
    }
}

bool bi_rrtPlanner::checkReachGoal(int id, node newForwrdNode)
{
    double distance;
    bool result = false;
    distance = calcDist(newForwrdNode, backwrdTree[id]);
    if(distance < minGoalDist){minGoalDist = distance;}
    if(distance > maxGoalDist){maxGoalDist = distance;}
    if(distance<=goalTolerance*JOINTNUM){
        result = true;
    }
    return result;
}

bool bi_rrtPlanner::plan()
{
    // Set some parameters i.e. this->success...
    initialize();

    // The main plan process, return true if successfully planned.
    int count = 0;
    while(this->success==false && count < maxIter){
        node randNode = sampleNode();
        // Forward tree extend
        int nearestNodeid = findNearest(randNode, forwrdTree);
        node newForNode = extend(nearestNodeid, randNode, 0);
        if(newForNode.id!=-1){  // Means the newForNode is feasible
            newForNode.cluster_id = 0;  // 0 - forward tree
            forwrdTree.push_back(newForNode);
            drawnewNode(newForNode);

            nearestNodeid = findNearest(newForNode, backwrdTree);
            if( checkReachGoal(nearestNodeid, newForNode) ){
                this->success = true;
            }else{
                node newBckNode = extend(nearestNodeid, newForNode, 1);
                if(newBckNode.id!=-1){
                    newBckNode.cluster_id = 1;  // 1 - backward tree
                    backwrdTree.push_back(newBckNode);
                    drawnewNode(newBckNode);
                }
            }            
        }

        cout << "Current node forwrdTree count: " << forwrdTree.size() << " backwrdTree: " << backwrdTree.size() << endl;
        count += 1;
    }
    if(this->success){
        findPath();
        drawPlan();
    }else{
        ROS_WARN("No available path found");
    }
    cout << "Minimum distance is: " << minGoalDist << endl;
    cout << "Maximum distance is: " << maxGoalDist << endl;
    return this->success;
}

void bi_rrtPlanner::initialize()
{
    // initialize the backwrdTree
    // Remember to add cluster_id to the nodes
    forwrdTree.clear();
    backwrdTree.clear();
    initialNode.cluster_id = 0;
    forwrdTree.push_back(initialNode);
    goalNodes[0].cluster_id = 1;
    backwrdTree.push_back(goalNodes[0]);

    path.clear();
    minGoalDist = DBL_MAX;
    maxGoalDist = 0;
    success = false;

    initrrtVisual();
    initBacktreeVisual();
    initPathVisual();
}

void bi_rrtPlanner::findPath()
{
    ;
}

void bi_rrtPlanner::drawnewNode(node newNode)
{
    // Differentiate backward and forward tree
    if(newNode.cluster_id==0){
        points.header.frame_id = line_list.header.frame_id = "/world";
        points.header.stamp = line_list.header.stamp = ros::Time::now();

        geometry_msgs::Point newPoint;
        geometry_msgs::Point prevPoint;
        calcNodePose(newNode, &newPoint);
        calcNodePose(forwrdTree[newNode.prevNodeid], &prevPoint);

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
        calcNodePose(backwrdTree[newNode.prevNodeid], &prevPoint);

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
    ;
}

void bi_rrtPlanner::initBacktreeVisual()
{
    points_bck.points.clear();
    line_list_bck.points.clear();

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