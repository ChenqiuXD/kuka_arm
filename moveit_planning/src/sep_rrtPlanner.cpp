#include "sep_rrtPlanner.h"

sep_rrtPlanner::sep_rrtPlanner(ros::NodeHandle& nh,
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
    markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // Readin joint limits
    readJntLimits();    
}


bool sep_rrtPlanner::plan()
{
    initialize();
    getSimplePath();
    
    // For display debugging utils only
    // drawSimplePathContinuity();
    // return false;

    seperatePaths();
    drawSimplePath();
    for(size_t i=goalGroupCount.size()-1;i>0;--i){  // Check whether some simple path can directly connect start and end pos
        if(goalGroupCount[i]-goalGroupCount[i-1]==1){
            cout << "Successfully connected in first step by connecting " << i << " and " << i+1 << " nodeGroups" << endl;
            this->success = true;
            addInTree(goalGroupCount[i]-1);
            break;
        }
    }
    if(!this->success){
        for(size_t i=startGroupid.size()-1;i>0;--i){      // Push all the start groups into the rrt search tree
            addInTree(startGroupid[i]);
            for(size_t j=0;j<goalGroupCount.size();++j){
                if(goalGroupCount[j]>=startGroupid[i]){
                    goalGroupCount[j] -= 1;
                }
            }
        }
    }

    int iterCount = 0;
    while( !this->success && iterCount<=this->maxIter ){
        node randNode = sampleNode();
        int nearestNodeid = findNearest(randNode, this->rrtTree);
        node newNode = extend(nearestNodeid, randNode);
        if(newNode.id != -1){   // -1 means that local planner could not find feasible path between nearest and newNode
            rrtTree.push_back(newNode);
            drawNewNode(newNode);

            int groupid, nodeid;
            checkConnection(newNode, &groupid, &nodeid);
            if(groupid){
                if(chechReachGoal(groupid)){
                    this->success = true;
                }
                cout << "Connecting to " << groupid << "th group's " << nodeid << "th node"<< endl; 
                connectToGroup(&newNode, groupid, nodeid);
            }
            cout << "Current iter number: " << iterCount << ", node number: " << rrtTree.size() << endl;
        }
        ++iterCount;
    }
    if(this->success){
        findPath();
        drawPlan();
    }

    return this->success;
}

void sep_rrtPlanner::initialize()
{
    this->simplePaths.clear();
    this->rrtTree.clear();
    this->nodeGroups.clear();
    this->path.clear();
    this->goalGroupCount.clear();
    this->startGroupid.clear();
    this->success = false;    

    this->initSimplePathVisual();
    this->initrrtVisual();
    this->initPathVisual();
}

void sep_rrtPlanner::getSimplePath()
{
    node curNode;
    curNode.id = 0;
    curNode.prevNodeid=-1;
    curNode.jointAngles = initialNode.jointAngles;

    // Cause jointAngles are radian (int), thus a stepAngles are used to calculate the 
    // steps of joints. The nodes in the simple path are mandatory formatted as int by the stepAngles
    double step = STEP * FEASI_PIESCES_NUM;
    vector<double> stepAngles;
    this->vecInt2Double(curNode.jointAngles, &stepAngles);  // Note that this is vector format change, not calculation unit change (degree to radian)
    double distance = getDistStartToEnd();

    int count = 0;
    double diff;
    vector<node> path0, path1, path2, path3, path4;
    while(abs(stepAngles[0])<=abs(goalNodes[0].jointAngles[0])){
        path0.push_back(curNode);
        diff = EXPAND_RATE * step * sin( step*count/distance*M_PI );
        vector<double> diffs = {+diff, +diff, -diff, +diff, -diff, +diff, 
                                -diff, +diff, -diff, -diff, -diff, +diff};
        vector<double> stepAnglesCopy = stepAngles;
        for(size_t i=0;i<4;++i){
            for(size_t j=0;j<3;++j){
                stepAnglesCopy[j] += diffs[i*3+j];
            }
            vecDoub2Int(stepAnglesCopy, &(curNode.jointAngles));
            stepAnglesCopy = stepAngles;
            if(i==0){path1.push_back(curNode);}
            else if(i==1){path2.push_back(curNode);}
            else if(i==2){path3.push_back(curNode);}
            else if(i==3){path4.push_back(curNode);}
        }

        for(size_t i=0;i<JOINTNUM;++i){
            stepAngles[i] += step / distance * (goalNodes[0].jointAngles[i]-initialNode.jointAngles[i]);
            curNode.jointAngles[i] = stepAngles[i];
        }
        ++count;
    }
    this->simplePaths.push_back(path0);
    this->simplePaths.push_back(path1);
    this->simplePaths.push_back(path2);
    this->simplePaths.push_back(path3);
    this->simplePaths.push_back(path4);
}

void sep_rrtPlanner::seperatePaths()
{
    this->groupCount = 0;
    for(size_t i=0;i<this->simplePaths.size();++i){
        this->seperateSimplePath(simplePaths[i]);
    }
}

void sep_rrtPlanner::seperateSimplePath(vector<node> &path)
{
    vector<node> group;
    for(size_t i=0;i<path.size()-1;++i){
        if(i==0){this->startGroupid.push_back(this->groupCount);}
        path[i].cluster_id = this->groupCount;
        group.push_back(path[i]);       
        if( !this->checkFeasbility(path[i], path[i+1]) ){
            do{
                // cout << "Obstacle in bewteen: " << i <<  " " << i+1 << endl;
                ++i;
            }while( i<path.size()-1 && !this->checkFeasbility(path[i], path[i+1]) );
            i -= 1; // To add the first non-colliding node
            this->groupCount += 1;
            this->nodeGroups.push_back(group);
            group.clear();
        }else{
            // cout << "No obstacle between " << i << " " << i+1 << endl;
            ;
        }
    }
    if(group.size()){
        this->groupCount += 1;
        this->nodeGroups.push_back(group);
        this->goalGroupCount.push_back(this->groupCount);
    }
}

void sep_rrtPlanner::addInTree(int nodeGroupid)
{
    nodeGroups[nodeGroupid][0].id = rrtTree.size();
    rrtTree.push_back(nodeGroups[nodeGroupid][0]);
    for(size_t i=1;i<nodeGroups[nodeGroupid].size();++i){
        nodeGroups[nodeGroupid][i].id = rrtTree.size();
        nodeGroups[nodeGroupid][i].prevNodeid = rrtTree.size()-1;
        rrtTree.push_back(nodeGroups[nodeGroupid][i]);
    }
    nodeGroups.erase(nodeGroups.begin()+nodeGroupid);
}

void sep_rrtPlanner::checkConnection(node newNode, int *groupid, int *nodeid)
{
    double dist, prevDist = DBL_MAX, minDist = DBL_MAX;
    size_t connectGroupid = 0, minNodeid = 0;
    for(size_t i=0;i<nodeGroups.size();++i){    // The first group of simple path has already pushed into rrtTree
        for(size_t j=0;j<nodeGroups[i].size();++j){
            dist = calcDist(newNode, nodeGroups[i][j]);
            if(dist>prevDist){  // If the distance is getting bigger, jump to next noedGroup
                break;
            }
            if(dist<minDist){
                minDist = dist;
                if(minDist < STEP*JOINTNUM){
                    connectGroupid = i;
                    minNodeid = j;
                }
            }
            prevDist = dist;
        }
    }
    *groupid = connectGroupid;
    *nodeid = minNodeid;
}

void sep_rrtPlanner::connectToGroup(node *newNode, int groupid, int nodeid)
{
    nodeGroups[groupid][nodeid].prevNodeid = newNode->id;
    nodeGroups[groupid][nodeid].id = rrtTree.size();
    rrtTree.push_back(nodeGroups[groupid][nodeid]);
    for(int i=nodeid-1;i>0;--i){
        nodeGroups[groupid][i].id = rrtTree.size();
        nodeGroups[groupid][i].prevNodeid = rrtTree.size()-1;
        rrtTree.push_back(nodeGroups[groupid][i]);
    }

    nodeGroups[groupid][nodeid+1].id = rrtTree.size();
    nodeGroups[groupid][nodeid+1].prevNodeid = nodeGroups[groupid][nodeid].id;
    rrtTree.push_back(nodeGroups[groupid][nodeid+1]);
    for(size_t i=nodeid+2;i<nodeGroups[groupid].size();++i){
        nodeGroups[groupid][i].id = rrtTree.size();
        nodeGroups[groupid][i].prevNodeid = rrtTree.size()-1;
        rrtTree.push_back(nodeGroups[groupid][i]);
    }
    nodeGroups.erase(nodeGroups.begin()+groupid);
}

bool sep_rrtPlanner::chechReachGoal(int groupid)
{
    if(goalGroupCount.size()){  // if there are goal node group
        bool isReach = false;
        for(size_t i=0;i<this->goalGroupCount.size();++i){
            if(groupid == this->goalGroupCount[i]-1){   // Check whether the groupid is one of the goal groups
                isReach = true;
                break;
            }
        }
        return isReach;
    }else{                      // if there are no goal node group, then use rrtPlanner check reach method
        bool result = true;
        node curNode = rrtTree.back();
        for( size_t j=0;j<JOINTNUM;++j ){
            if( abs(curNode.jointAngles[j] -goalNodes[0].jointAngles[j]) >this->goalToleranceVec[j] ){
                result = false;
                break;
            }
        }
        return result;
    }
    
}

void sep_rrtPlanner::findPath()
{
    // int curNodeid = findNearest(goalNodes[0], rrtTree);
    // node curNode = rrtTree[curNodeid];
    node curNode = rrtTree.back();
    while(curNode.prevNodeid!=-1){
        this->path.push_back(curNode);
        curNode = rrtTree[curNode.prevNodeid];
    }
}

// Utils
double sep_rrtPlanner::getDistStartToEnd()
{
    double distance;
    for(size_t i=0;i<JOINTNUM;++i){
        distance += pow(goalNodes[0].jointAngles[i]-initialNode.jointAngles[i],2);
    }
    distance = sqrt(distance);
    return distance;
}

void sep_rrtPlanner::drawSimplePathContinuity()
{
    geometry_msgs::Point point;
    geometry_msgs::Point nextPoint;
    for(auto groupIter = simplePaths.begin(); groupIter!=simplePaths.end();++groupIter){
        for(auto nodeIter = groupIter->begin();nodeIter!=groupIter->end()-2;++nodeIter){
            calcNodePose((*nodeIter), &point);
            calcNodePose(*(nodeIter+1), &nextPoint);
            this->points_simple.points.push_back(point);
            this->line_list_simple.points.push_back(point);
            this->line_list_simple.points.push_back(nextPoint);
        }
        this->points_simple.points.push_back(nextPoint);
    }

    points_simple.header.frame_id = line_list_simple.header.frame_id = "/world";
    points_simple.header.stamp = line_list_simple.header.stamp = ros::Time::now();
    points_simple.ns = line_list_simple.ns = "simple_paths";

    markerPub.publish(points_simple);
    markerPub.publish(line_list_simple);
}

void sep_rrtPlanner::drawSimplePath()
{
    geometry_msgs::Point point;
    geometry_msgs::Point nextPoint;

    for(size_t i=0;i<nodeGroups.size();++i){
        cout << nodeGroups[i].size() << endl;
    }

    for(auto groupIter = nodeGroups.begin(); groupIter!=nodeGroups.end();++groupIter){
        if(groupIter->size()<2)
            continue;
        for(auto nodeIter = groupIter->begin();nodeIter!=groupIter->end()-1;++nodeIter){
            calcNodePose((*nodeIter), &point);
            calcNodePose(*(nodeIter+1), &nextPoint);
            this->points_simple.points.push_back(point);
            this->line_list_simple.points.push_back(point);
            this->line_list_simple.points.push_back(nextPoint);
        }
        this->points_simple.points.push_back(nextPoint);
    }

    points_simple.header.frame_id = line_list_simple.header.frame_id = "/world";
    points_simple.header.stamp = line_list_simple.header.stamp = ros::Time::now();
    points_simple.ns = line_list_simple.ns = "simple_path";

    markerPub.publish(points_simple);
    markerPub.publish(line_list_simple);
}

void sep_rrtPlanner::initSimplePathVisual()
{
    points_simple.points.clear();
    line_list_simple.points.clear();

    points_simple.action = line_list_simple.action = visualization_msgs::Marker::ADD;
    points_simple.pose.orientation.w = line_list_simple.pose.orientation.w = 1.0;
    points_simple.id = 2;
    line_list_simple.id = 3;
    points_simple.type = visualization_msgs::Marker::POINTS;
    line_list_simple.type = visualization_msgs::Marker::LINE_LIST;

    points_simple.scale.x = 0.002;
    points_simple.scale.y = 0.002;
    line_list_simple.scale.x = 0.001;

    points_simple.color.r = 0.6f;
    points_simple.color.a = 1.0f;
    line_list_simple.color.r = 0.84f;
    line_list_simple.color.g = 0.8f;
    line_list_simple.color.b = 0.8f;
    line_list_simple.color.a = 1.0;
}

void sep_rrtPlanner::drawPlan()
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
    }
    ros::WallDuration sleep_t(0.1);
    markerPub.publish(pathEdges);
    sleep_t.sleep();
    markerPub.publish(pathVertices);
    sleep_t.sleep();
}