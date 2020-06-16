#include "bi_rrtMultPlanner.h"

bi_rrtMultPlanner::bi_rrtMultPlanner(ros::NodeHandle& nh,
                             robot_model::RobotModelPtr kinematic_model,
                             planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_)
                             : bi_rrtPlanner(nh, kinematic_model, planning_scene_monitor_)
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

bool bi_rrtMultPlanner::plan()
{
    initialize();

    // first step get the simple paths and push feasible ones into forward and backward tree.
    getSimplePath();
    seperateSimplePaths();
    addToTree();

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

void bi_rrtMultPlanner::getSimplePath()
{
    this->simplePaths.clear();
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

void bi_rrtMultPlanner::seperateSimplePaths()
{
    this->groupCount = 0;
    this->nodeGroups.clear();
    this->startGroupid.clear();
    this->goalGroupCount.clear();
    for(size_t i=0;i<this->simplePaths.size();++i){
        this->seperatePath(simplePaths[i]);
    }
}

void bi_rrtMultPlanner::seperatePath(vector<node> &path)
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

void bi_rrtMultPlanner::addToTree()
{
    for(size_t i=0;i<startGroupid.size();++i){
        int groupid = startGroupid[i];
        for(size_t j=0;j<nodeGroups[groupid].size();++j){
            if(j==0){
                nodeGroups[groupid][j].prevNodeid = 0;
            }else{
                nodeGroups[groupid][j].prevNodeid = forwardTree.size()-1;
            }
            nodeGroups[groupid][j].id = forwardTree.size();
            nodeGroups[groupid][j].cluster_id = 0;
            forwardTree.push_back(nodeGroups[groupid][j]);
            drawNewNode(nodeGroups[groupid][j]);
        }
    }
    for(size_t i=0;i<goalGroupCount.size();++i){
        int groupid = goalGroupCount[i]-1;
        for(int j = nodeGroups[groupid].size()-1;j>0;--j){
            if(j==nodeGroups[groupid].size()-1){
                nodeGroups[groupid][j].prevNodeid = 0;
            }else{
                nodeGroups[groupid][j].prevNodeid = backwardTree.size()-1;
            }
            nodeGroups[groupid][j].id = backwardTree.size();            
            nodeGroups[groupid][j].cluster_id = 1;
            backwardTree.push_back(nodeGroups[groupid][j]);
            drawNewNode(nodeGroups[groupid][j]);
        }
    }
}

double bi_rrtMultPlanner::getDistStartToEnd()
{
    double distance;
    for(size_t i=0;i<JOINTNUM;++i){
        distance += pow(goalNodes[0].jointAngles[i]-initialNode.jointAngles[i],2);
    }
    distance = sqrt(distance);
    return distance;
}

// void bi_rrtMultPlanner::findPath()
// {
//     ;
// }

// void bi_rrtMultPlanner::drawPlan()
// {
//     ;
// }