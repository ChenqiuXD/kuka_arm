#include "rrt.h"

bool RRT::planMultConnect()
{
    initMultConnect();
    if(getSimplePath()){return true;}

    bool result = false, isGrowBackTree = false;
    for(int i=0;i<this->max_iter;++i){
        Node *q = getRandomNode();
        if(isGrowBackTree){
            growBackTree(q);
        }else{
            growForwardTree(q);
        }
        if(checkReachGoal()){
            result = true;
            break;
        }
        isGrowBackTree = !isGrowBackTree;
    }
    return result;
}

void RRT::initMultConnect()
{
    this->tempNodes.clear();
    this->backTree.clear();
}

bool RRT::getSimplePath()
{
    bool result;
    getForPath();
    if((*(tempNodes.end()-1))->position(0) >= goalPos(0)){
        result = true;
    }else{
        result = false;
    }
    getBackPath();
    return result;
}

void RRT::getForPath()
{
    Node *curNode = new Node;
    curNode->parent = NULL;
    curNode->position = startPos;
    Vector2f intermediate = endPos - startPos;
    Vector2f step = this->step_size * intermediate / intermediate.norm();

    Node *nextNode = new Node;
    nextNode->position = curNode->position + step;
    while(curNode->position(0) < goalPos(0) &&
          !this->obstacles->isSegmentInObstacle(curNode->position, nextNode->position)){
        nextNode->parent = curNode;
        curNode->children.push_back(nextNode);
        this->tempNodes.push_back(curNode);
        curNode = nextNode;
        nextNode = new Node;
        nextNode->position = curNode->position + step;
    }
}

void RRT::getBackPath()
{
    Node *curNode = new Node;
    curNode->parent = NULL;
    curNode->position = endPos;
    Vector2f intermediate = endPos - startPos;
    Vector2f step = this->step_size * intermediate / intermediate.norm();

    Node *prevNode = new Node;
    prevNode->position = curNode->position - step;
    while(curNode->position(0) > startPos(0) &&
          !this->obstacles->isSegmentInObstacle(curNode->position, prevNode->position)){
        prevNode->parent = curNode;
        curNode->children.push_back(prevNode);
        this->backTree.push_back(curNode);
        curNode = prevNode;
        prevNode = new Node;
        prevNode->position = curNode->position - step;
    }
}

void RRT::findPathMultConnect(bool success)
{
    Node *qFor, *qBack;
    if(success){
        qFor = this->lastNode;
        qBack = this->lastBackNode;
    }else{
        qFor = nearest(this->endPos);
        qBack = nearest(this->startPos, this->backTree);
    }

    while(qFor!=NULL){
        this->path.push_back(qFor);
        qFor = qFor->parent;
    }
    while(qBack!=NULL){
        this->path.push_back(qBack);
        qBack = qBack->parent;
    }
}
