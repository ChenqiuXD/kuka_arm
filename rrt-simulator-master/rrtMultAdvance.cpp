#include "rrt.h"

bool RRT::planMultAdvance()
{
    this->success = false;
    initMultAdvance();
    getSimplePaths();
    seperatePaths();
    this->goalNode =*((nodeGroups.end()-1)->end()-1);

    // for(size_t i=0;i<this->nodeGroups.size();++i){
    //     nodes.insert(nodes.begin(), nodeGroups[i].begin(), nodeGroups[i].end());
    // }
    // cout << nodes.size() << endl;
    // return true;

    this->groupCount = nodeGroups.size()-1;
    if(this->nodeGroups.size()==1){this->success = true;}
    this->tempNodes = nodeGroups[0];

    int iterCount = 0;
    while( !this->success && iterCount <= this->max_iter ){
        Node *q = getRandomNode();
        if(q){
            Node *qNearest = nearest(q->position);
            Vector2f newPos = newConfig(q, qNearest);
            if(!isSegInObstacle(q, qNearest)){
                Node *qNew = new Node;
                qNew->position = newPos;
                add(qNearest, qNew);

                int groupid, nodeid;
                checkConnection(&groupid, &nodeid);
                if(groupid){
                    if(groupid==this->groupCount){
                        this->success = true;
                    }
                    connectToGroup(groupid, nodeid);
                    this->groupCount -= 1;
                }
            }
        }

        ++iterCount;
    }

    return this->success;
}

void RRT::initMultAdvance()
{
    this->simplePaths.clear();
    this->tempNodes.clear();
    this->nodeGroups.clear();
}

void RRT::findPathMultAdvance(bool)
{
    int iterCount = 0;
    Node *q;
    if(this->success){
        q = this->goalNode;
    }else{
        q = nearest(this->endPos);
    }

    while(iterCount <= max_iter && q!=NULL){
        ++iterCount;
        path.push_back(q);
        q = q->parent;
    }
}

void RRT::getSimplePaths()
{
    // First compute the five interval points
    int expandCoff = 4;
    double intermediate = (goalPos[1] - startPos[1])/2;
    double intervalStep = intermediate / expandCoff;
    vector<int> coefVec{0, -1, 1, -2, 2};
    vector<Vector2f> intervalPoint;
    for( int &coef : coefVec ){
        Vector2f intervalPos(intermediate + coef*intervalStep,
                             intermediate - coef*intervalStep);
        intervalPoint.push_back(intervalPos);
    }

    // Generate five simple paths
    vector<Node *> simplePath;
    for(size_t i=0;i<intervalPoint.size();++i){
        Node *curNode = new Node;
        curNode->parent = NULL;
        curNode->position = startPos;
        Vector2f intermediate = intervalPoint[i] - startPos;
        intermediate = intermediate / intermediate.norm();
        while(curNode->position[1] <= intervalPoint[i][1]){
            Node *nextNode = new Node;
            nextNode->parent = curNode;
            curNode->children.push_back(nextNode);
            nextNode->position = curNode->position + step_size * intermediate;
            simplePath.push_back(curNode);
            curNode = nextNode;
        }

        intermediate = goalPos - intervalPoint[i];
        intermediate = intermediate / intermediate.norm();
        while(curNode->position[1] <= goalPos[1]){
            Node *nextNode = new Node;
            nextNode->parent = curNode;
            curNode->children.push_back(nextNode);
            nextNode->position = curNode->position + step_size * intermediate;
            simplePath.push_back(curNode);
            curNode = nextNode;
        }
        this->simplePaths.push_back(simplePath);
        simplePath.clear();
    }
}

void RRT::seperatePaths()
{
    vector< vector<Node *>> sepSimplePath;
    unsigned minBlockNum=1e9;
    for(size_t i=0;i<this->simplePaths.size();++i){
        sepSimplePath = dividePath(simplePaths[i]);
        if(sepSimplePath.size() < minBlockNum){
            minBlockNum = sepSimplePath.size();
            this->nodeGroups = sepSimplePath;
        }
        sepSimplePath.clear();
    }
}

vector<vector<Node *>> RRT::dividePath(vector<Node *> path)
{
    vector<Node *> group;
    vector<vector<Node *>> nodeGroups;
    for(size_t i=0;i<path.size()-1;++i){
        group.push_back(path[i]);
        if(this->isSegInObstacle(path[i], path[i+1])){
            do{
                ++i;
            }while(i<path.size()-2 && !this->isSegInObstacle(path[i], path[i+1]));
            path[i+1]->parent = NULL;
            nodeGroups.push_back(group);
            group.clear();
        }
    }
    nodeGroups.push_back(group);
    return nodeGroups;
}

void RRT::checkConnection(int *groupid, int *nodeid)
{
    int dist, prevDist=1e9, minDist = 1e9;
    size_t connectGroupid = 0, minNodeid = 0;
    for(size_t i=1; i<nodeGroups.size(); ++i){
        for(size_t j=0; j<nodeGroups[i].size(); ++j){
            dist = distance(lastNode->position, nodeGroups[i][j]->position);
            if(dist<minDist){
                minDist = dist;
                if(minDist<step_size){
                    connectGroupid = i;
                    minNodeid = j;
                }
            }
            if(dist>prevDist){break;}
            prevDist = dist;
        }
    }
    *groupid = connectGroupid;
    *nodeid = minNodeid;
}

void RRT::connectToGroup(int groupid, int nodeid)
{
    if(nodeid!=0){
        swapRelation(groupid, nodeid);
    }else{
        nodeGroups[groupid][nodeid]->parent = this->lastNode;
    }
    tempNodes.insert(tempNodes.end(),
                     nodeGroups[groupid].begin(),
                     nodeGroups[groupid].end());
    nodeGroups.erase(nodeGroups.begin()+groupid);
}

void RRT::swapRelation(int groupid, int nodeid)
{
    for( size_t nodeCount=nodeid-1; nodeCount > 0; --nodeCount ){
        nodeGroups[groupid][nodeCount]->children.clear();
        nodeGroups[groupid][nodeCount]->children.push_back(nodeGroups[groupid][nodeCount-1]);
        nodeGroups[groupid][nodeCount]->parent = nodeGroups[groupid][nodeCount+1];
    }
    nodeGroups[groupid][0]->parent = nodeGroups[groupid][1];
    nodeGroups[groupid][0]->children.clear();
    nodeGroups[groupid][nodeid]->parent = lastNode;
}
