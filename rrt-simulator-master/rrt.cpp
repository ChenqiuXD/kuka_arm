#include "rrt.h"

RRT::RRT()
{
    obstacles = new Obstacles;
    startPos.x() = START_POS_X;
    startPos.y() = START_POS_Y;
    endPos.x() = END_POS_X;
    endPos.y() = END_POS_Y;
    goalPos.x() = END_POS_X;
    goalPos.y() = END_POS_Y;
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    tempNodes.push_back(root);
    nodes.push_back(root);
    step_size = 10;
    max_iter = 3000;
}

/**
 * @brief plan loop of RRT
 */
bool RRT::plan()
{
    bool result = false;
    for(int i = 0; i < this->max_iter; i++) {
        Node *q = getRandomNode();
        if (q) {
            Node *qNearest = nearest(q->position);
            if (this->distance(q->position, qNearest->position) > this->step_size) {
                Vector2f newPos = newConfig(q, qNearest);
                if (!this->obstacles->isSegmentInObstacle(newPos, qNearest->position)) {
                    Node *qNew = new Node;
                    qNew->position = newPos;
                    add(qNearest, qNew);
                }
            }
        }
        if (this->reached()) {
            result = true;
            break;
        }
        // cout << "This is the " << i << "th iteration" << endl;
    }

    return result;
}

void RRT::findPath(bool result)
{
    Node *q;
    if (result) {
        q = this->lastNode;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = nearest(this->endPos);
    }

    while (q != NULL) {
        this->path.push_back(q);
        q = q->parent;
    }
}

/**
 * @brief Initialize root node of RRT.
 */
void RRT::initialize()
{
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    tempNodes.push_back(root);
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRT::getRandomNode()
{
    Node* ret;
    Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
    if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT) {
        ret = new Node;
        ret->position = point;
        // cout << "random points: " << ret->position[0] << " " << ret->position[1] << endl;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
int RRT::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRT::nearest(Vector2f point)
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

Node* RRT::nearest(Vector2f point, vector<Node*> tree)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)tree.size(); i++) {
        float dist = distance(point, tree[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = tree[i];
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
Vector2f RRT::newConfig(Node *q, Node *qNearest)
{
    Vector2f to = q->position;
    Vector2f from = qNearest->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f ret = from + step_size * intermediate;
    return ret;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRT::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    tempNodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRT::reached()
{
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
        return true;
    return false;
}

void RRT::setStepSize(int step)
{
    step_size = step;
}

void RRT::setMaxIterations(int iter)
{
    max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRT::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}

// -------------------------------------------------------------------------
// Following code is written as a modified version of RRT as a demonstration
// And is the method 2
// -------------------------------------------------------------------------

/**
 * @brief Second plan methods. Generate an simple plan and connect those that are blocked
 */
bool RRT::planConnect()
{
    this->initConnect();
    this->getSimplePlan();
    this->getBlockedNodes();
    bool success = true;
    for(size_t i=0;i<this->blockedNodes.size();i+=2){
        cout << "Connecting " << i << "th blocked node with " << i+1 << "th blocked node" << endl;
        this->setInitNode(this->blockedNodes[i]);
        this->setEndNode(this->blockedNodes[i+1]->position);
        success = this->plan();
        if( !success ){
            cout << "Failed to connect " << i << " and " << i+1 << " nodes, thus stop planning" << endl;
            nodes.insert(nodes.end(), tempNodes.begin(), tempNodes.end());
            break;
        }
        else{
            cout << "Successfully connect " << i << " and " << i+1 << " nodes. " << endl;
            this->lastNode->children.push_back(blockedNodes[i+1]);
            blockedNodes[i+1]->parent = this->lastNode;
            nodes.insert(nodes.end(), tempNodes.begin(), tempNodes.end());
            tempNodes.clear();
        }
    }
    findPathConnect(success);
    this->root = *(this->simplePath.begin());
    return success;
}

void RRT::initConnect()
{
    this->simplePath.clear();
    this->blockedNodes.clear();
}

/**
 * @brief Generate a simple plan by directly linking start and end
 */
void RRT::getSimplePlan()
{
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
        // cout << "New point is: " << curNode->position(0) << " " << curNode->position(1) << endl;
        curNode = nextNode;
    }
}

/**
 * @brief Get the pairs of nodes. 
 *      The first is the node before blocked and the second is the free node
 *      after many blocked nodes
 */
void RRT::getBlockedNodes()
{
    for(size_t i=0;i<this->simplePath.size()-1;++i){
        this->nodes.push_back(simplePath[i]);
        if( this->isSegInObstacle(simplePath[i], simplePath[i+1]) ){
            this->blockedNodes.push_back(simplePath[i]);
            do{
                cout << "Obstacle in between: " << i << " " << i+1 << endl;
                ++i;
            }while( i<simplePath.size() && !this->isSegInObstacle(simplePath[i], simplePath[i+1]) );
            if(i<=simplePath.size()-1){
                this->blockedNodes.push_back(simplePath[i+1]);
            }
        }else{
            cout << "No obstacle in between " << i << " " << i+1 << endl;
            ;
        }
    }
}

void RRT::findPathConnect(bool result)
{
    if(result){
        Node *q;
        q = *(this->simplePath.end()-1);
        while(q){
            this->path.push_back(q);
            q = q->parent;
        }
    }
}

bool RRT::isSegInObstacle(Node *a, Node *b)
{
    return this->obstacles->isSegmentInObstacle(a->position, b->position);
}

/**
 * @brief set the initial node
 */
void RRT::setInitNode(Node *node)
{
    root = node;
    lastNode = root;
    tempNodes.push_back(root);
}

/**
 * @brief set the end node
 */
void RRT::setEndNode(Vector2f point)
{
    this->endPos = point;
}

