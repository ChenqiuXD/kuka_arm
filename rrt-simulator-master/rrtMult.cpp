#include "rrtMult.h"


RRTMult::RRTMult()
{
    obstacles = new Obstacles;
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;
    goalPos.x() = END_POS_X;
    goalPos.y() = END_POS_Y;
    step_size = 8;
    max_iter = 3000;
}

/**
 * @brief plan loop of RRT
 */
bool RRTMult::plan()
{
    this->success = false;
    initialize();
    getSimplePath();
    seperateSimplePath();
    
    // for(int i=0;i<this->nodeGroups.size();++i){
    //     this->nodes.insert(nodes.end(), nodeGroups[i].begin(), nodeGroups[i].end());
    // }
    // return false;

    this->groupCount = this->nodeGroups.size()-1;
    this->tempNodes = nodeGroups[0];
    if(this->nodeGroups.size()==1){ //means that there are no obstacles on the line between start and end
        this->success = true;
    }

    int iterCount = 0;
    while(!this->success && iterCount<=this->max_iter){
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
    findPath();
    return this->success;
}

/**
 * @brief Initialize root node of RRT.
 */
void RRTMult::initialize()
{
    this->simplePath.clear();
    this->tempNodes.clear();
    this->nodeGroups.clear();
}

/**
 * @brief Generate a simple plan by directly linking start and end
 */
void RRTMult::getSimplePath()
{
    if(simplePath.size()){return;}  // If simple path already exist, skip
    Node *curNode = new Node;
    curNode->parent = NULL;
    curNode->position = startPos;
    Vector2f intermediate = endPos - startPos;
    Vector2f step = this->step_size * intermediate / intermediate.norm();

    while(curNode->position(0) < goalPos(0)){
        Node *nextNode = new Node;
        nextNode->parent = curNode;
        curNode->children.push_back(nextNode);
        nextNode->position = curNode->position + step;
        this->simplePath.push_back(curNode);
        curNode = nextNode;
    }
}

/**
 * @brief Get the groups of nodes
 *      Seperate the simple path nodes with obstacles and group them
 *      into this->nodeGroups
 */
void RRTMult::seperateSimplePath()
{
    this->groupCount = 0;
    vector<Node *> group;
    for(size_t i=0; i<this->simplePath.size()-1; ++i){
        group.push_back(simplePath[i]);
        if( this->isSegInObstacle(simplePath[i], simplePath[i+1]) ){
            do{
                ++i;
            }while( !this->isSegInObstacle(simplePath[i], simplePath[i+1]) && i<this->simplePath.size()-2);
            simplePath[i+1]->parent = NULL;
            this->nodeGroups.push_back(group);
            group.clear();
        }else{
        }
    }
    nodeGroups.push_back(group);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRTMult::getRandomNode()
{
    Node* ret;
    Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT) {
        ret = new Node;
        ret->position = point;
        return ret;
    }
    return NULL;
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRTMult::nearest(Vector2f point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)tempNodes.size(); i++) {
        float dist = distance(point, tempNodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = tempNodes[i];
        }
    }
    return closest;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Vector2f RRTMult::newConfig(Node *q, Node *qNearest)
{
    int dist = this->distance(q->position, qNearest->position);
    Vector2f ret;
    if(dist>this->step_size){
        Vector2f to = q->position;
        Vector2f from = qNearest->position;
        Vector2f intermediate = to - from;
        intermediate = intermediate / intermediate.norm();
        ret = from + step_size * intermediate;
    }else{
        ret = q->position;
    }
    return ret;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTMult::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    tempNodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Add a node to the tree.
 * @param groupid   : store the connected group num
 * @param nodeid    : store the connected node num
 */
void RRTMult::checkConnection(int *groupid, int *nodeid)
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

/**
 * @brief Add a node to the tree.
 * @param groupid   : store the connected group num
 * @param nodeid    : store the connected node num
 */
void RRTMult::connectToGroup(int groupid, int nodeid)
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

/**
 * @brief Swap the parent children relationship from the connected node
 * @param groupid   : store the connected group num
 * @param nodeid    : store the connected node num
 */
void RRTMult::swapRelation(int groupid, int nodeid)
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

/**
 * @brief Find path if successful, otherwise find the nearest point
 * @return
 */
void RRTMult::findPath()
{
    int iterCount = 0;
    Node *q;
    if(this->success){
        q = *(simplePath.end()-1);
    }else{
        q = nearest(this->endPos);
    }

    while(iterCount <= max_iter && q!=NULL){
        ++iterCount;
        path.push_back(q);
        q = q->parent;
    }
}

// Utils functions
/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
int RRTMult::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

void RRTMult::setStepSize(int step)
{
    step_size = step;
}

void RRTMult::setMaxIterations(int iter)
{
    max_iter = iter;
}

/**
 * @brief Check whether two nodes intersect the border of any obstacles
 *      Note that this function only check for intersection. If two nodes
 *      are both in the obstacles, the function would not detect collision
 * @param a
 * @param b
 * @return Collided or not
 */
bool RRTMult::isSegInObstacle(Node *a, Node *b)
{
    return this->obstacles->isSegmentInObstacle(a->position, b->position);
}

