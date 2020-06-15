#include "rrtConnectPlanner.h"

rrtConnectPlanner::rrtConnectPlanner(ros::NodeHandle& nh,
                             robot_model::RobotModelPtr kinematic_model,
                             planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_)
                             : bi_rrtPlanner(nh, kinematic_model, planning_scene_monitor_)
{
    // rrtPlanner(nh, kinematic_model, planning_scene_monitor_);
    this->kinematicModel = kinematic_model;
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    this->kinematicState = kinematic_state;
    this->kinematicState->setToDefaultValues();
    this->jointModelGroup = kinematic_model->getJointModelGroup("manipulator");
    this->planningSceneMonitor_ = planning_scene_monitor_;

    // Rviz display publisher
    this->markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // Readin joint limits
    readJntLimits();  
}

bool rrtConnectPlanner::plan()
{
    initialize();

    // main loop
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

void rrtConnectPlanner::initialize()
{
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
    success = false;

    initrrtVisual();
    initBacktreeVisual();
    initPathVisual();
}

void rrtConnectPlanner::growFrdTree(node randNode)
{
    int nearestNodeid = findNearest(randNode, forwardTree);
    node newFrdNode = extend(nearestNodeid, randNode, 0);
    if(newFrdNode.id != -1){    // if this node is feasible
        newFrdNode.cluster_id = 0;
        forwardTree.push_back(newFrdNode);
        lastFrdNode = newFrdNode;
        drawNewNode(newFrdNode);

        nearestNodeid = findNearest(newFrdNode, backwardTree);
        connect(nearestNodeid, newFrdNode, 1);
    }
}

void rrtConnectPlanner::growBackTree(node randNode)
{
    int nearestNodeid = findNearest(randNode, backwardTree);
    node newBckNode = extend(nearestNodeid, randNode, 1);
    if(newBckNode.id != -1){    // if this node is feasible
        newBckNode.cluster_id = 1;
        backwardTree.push_back(newBckNode);
        lastBackNode = newBckNode;
        drawNewNode(newBckNode);

        nearestNodeid = findNearest(newBckNode, forwardTree);
        connect(nearestNodeid, newBckNode, 0);
    }
}

void rrtConnectPlanner::connect(int nearestNodeid, node lastNode, int cluster_id)
{
    node nearestNode;
    if(cluster_id==0){nearestNode = forwardTree[nearestNodeid];}
    else if(cluster_id==1){nearestNode = backwardTree[nearestNodeid];} 

    int iterCount = 0;
    bool isFeasible = true, isReached = false;
    node newNode;
    while(iterCount <= MAX_ITER_CONNECT && isFeasible && !isReached){
        newNode.jointAngles = newJoints(lastNode, nearestNode);
        isFeasible = checkFeasbility(nearestNode, newNode);
        if(isFeasible){
            newNode.prevNodeid = nearestNode.id;
            if(cluster_id==0){
                newNode.id = forwardTree.size();
                newNode.cluster_id = 0;
                forwardTree.push_back(newNode);
                lastFrdNode = newNode;
            }else{
                newNode.id = backwardTree.size();
                newNode.cluster_id = 1;    
                backwardTree.push_back(newNode);
                lastBackNode = newNode;
            }
            drawNewNode(newNode);
            nearestNode = newNode;
            isReached = checkReachGoal();
        }

        ++iterCount;
    }
}

vector<int> rrtConnectPlanner::newJoints(node to, node from)
{
    double distance = calcDist(to, from);
    double step = STEP*JOINTNUM;
    if(distance <= step){
        return to.jointAngles;
    }else{
        vector<int> angles;
        for(int i=0;i<JOINTNUM;++i){
            int newAngle = int(from.jointAngles[i] + (to.jointAngles[i]-from.jointAngles[i])*step/distance);
            angles.push_back(newAngle);
        }
        return angles;
    }
}
