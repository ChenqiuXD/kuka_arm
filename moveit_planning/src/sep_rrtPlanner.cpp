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
    seperateSimplePath();
    drawSimplePath();
    this->rrtTree.insert(rrtTree.end(), nodeGroups[0].begin(), nodeGroups[0].end());
    if(this->nodeGroups.size()==1){
        this->success = true;
    }

    int iterCount = 0;
    while( !this->success && iterCount<=this->maxIter ){
        node randNode = sampleNode();
        int nearestNodeid = findNearest(randNode, this->rrtTree);
        node newNode = extend(nearestNodeid, randNode);
        if(newNode.id != -1){   // -1 means that local planner could not find feasible path between nearest and newNode
            int groupid, nodeid;
            checkConnection(newNode, &groupid, &nodeid);
            if(groupid){
                if(groupid==this->groupCount){
                    this->success = true;
                }
                cout << "Connecting to " << groupid << "th group's " << nodeid << "th node"<< endl; 
                connectToGroup(&newNode, groupid, nodeid);
            }

            rrtTree.push_back(newNode);
            drawNewNode(newNode);
            cout << "Current iter number: " << iterCount << ", node number: " << rrtTree.size() << endl;
        }
        ++iterCount;
    }
    if(this->success){
        findPath();
    }
    return this->success;
}

void sep_rrtPlanner::initialize()
{
    this->simplePath.clear();
    this->rrtTree.clear();
    this->nodeGroups.clear();
    this->path.clear();

    this->initVisual();
    this->initrrtVisual();
}

void sep_rrtPlanner::getSimplePath()
{
    node curNode;
    curNode.id = 0;
    curNode.prevNodeid=-1;
    curNode.jointAngles = initialNode.jointAngles;

    double step = STEP * FEASI_PIESCES_NUM;
    vector<double> stepAngles;
    this->vecInt2Double(curNode.jointAngles, &stepAngles);
    double distance = getDistStartToEnd();

    int count = 0;
    while(abs(curNode.jointAngles[0])<=abs(goalNodes[0].jointAngles[0])){
        this->simplePath.push_back(curNode);
        curNode.prevNodeid = count;
        ++count;
        curNode.id = count;
        for(size_t i=0;i<JOINTNUM;++i){
            stepAngles[i] += step / distance * (goalNodes[0].jointAngles[i]-initialNode.jointAngles[i]);
            curNode.jointAngles[i] = stepAngles[i];
            cout << "The" << count << "th node's " << i << "th joint angle is: " << curNode.jointAngles[i] << endl;
        }
    }
}

void sep_rrtPlanner::seperateSimplePath()
{
    this->groupCount = 0;
    vector<node> group;
    for(size_t i=0;i<this->simplePath.size()-1;++i){
        simplePath[i].cluster_id = this->groupCount;
        group.push_back(simplePath[i]);       
        cout << simplePath.size() << endl; 
        if( !this->checkFeasbility(simplePath[i], simplePath[i+1]) ){
            do{
                cout << "Obstacle in bewteen: " << i <<  " " << i+1 << endl;
                ++i;
            }while( i<simplePath.size()-1 && !this->checkFeasbility(simplePath[i], simplePath[i+1]) );
            i -= 1; // To add the first non-colliding node
            this->groupCount += 1;
            this->nodeGroups.push_back(group);
            group.clear();
        }else{
            cout << "No obstacle between " << i << " " << i+1 << endl;
        }
    }
    this->nodeGroups.push_back(group);
}

void sep_rrtPlanner::checkConnection(node newNode, int *groupid, int *nodeid)
{
    double dist, minDist = DBL_MAX;
    size_t connectGroupid = 0, minNodeid = 0;
    for(size_t i=1;i<nodeGroups.size();++i){    // The first group of simple path has already pushed into rrtTree
        for(size_t j=0;j<nodeGroups[i].size();++j){
            dist = calcDist(newNode, nodeGroups[i][j]);
            if(dist<minDist){
                minDist = dist;
                if(minDist < STEP*JOINTNUM*FEASI_PIESCES_NUM){
                    connectGroupid = i;
                    minNodeid = j;
                }
            }
        }
    }
    *groupid = connectGroupid;
    *nodeid = minNodeid;
}

void sep_rrtPlanner::connectToGroup(node *newNode, int groupid, int nodeid)
{
    nodeGroups[groupid][nodeid].prevNodeid = newNode->id;
    rrtTree.insert(rrtTree.end(), nodeGroups[groupid].begin(), nodeGroups[groupid].end());
    nodeGroups.erase(nodeGroups.begin()+groupid);
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

void sep_rrtPlanner::drawSimplePath()
{
    geometry_msgs::Point point;
    geometry_msgs::Point nextPoint;
    for(auto groupIter = nodeGroups.begin(); groupIter!=nodeGroups.end();++groupIter){
        for(auto nodeIter = groupIter->begin();nodeIter!=groupIter->end()-2;++nodeIter){
            calcNodePose((*nodeIter), &point);
            calcNodePose(*(nodeIter+1), &nextPoint);
            this->points_simple.points.push_back(point);
            this->line_list_simple.points.push_back(point);
            this->line_list_simple.points.push_back(nextPoint);
        }
    }

    points_simple.header.frame_id = line_list_simple.header.frame_id = "/world";
    points_simple.header.stamp = line_list_simple.header.stamp = ros::Time::now();
    points_simple.ns = line_list_simple.ns = "simple_path";

    markerPub.publish(points_simple);
    markerPub.publish(line_list_simple);
}

void sep_rrtPlanner::initVisual()
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

    points_simple.color.g = 1.0f;
    points_simple.color.a = 1.0f;
    line_list_simple.color.r = 1.0f;
    line_list_simple.color.a = 1.0;
}