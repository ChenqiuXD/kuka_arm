#include "rrt.h"

bool RRT::planBiConnect()
{
    initBiConnect();
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

void RRT::growBackTree(Node *qRand)
{
    if(qRand){
        Node *qNearest = nearest(qRand->position, this->backTree);
        Vector2f newPos = newConfig(qRand, qNearest);
        if(!this->obstacles->isSegmentInObstacle(newPos, qNearest->position)){
            Node *qNew = new Node;
            qNew->position = newPos;
            addInBack(qNearest, qNew);
            connectToBackNode();
        }
    }
}

void RRT::growForwardTree(Node *qRand)
{
    if(qRand){
        Node *qNearest = nearest(qRand->position);
        Vector2f newPos = newConfig(qRand, qNearest);
        if(!this->obstacles->isSegmentInObstacle(newPos, qNearest->position)){
            Node *qNew = new Node;
            qNew->position = newPos;
            add(qNearest, qNew);
            connectToForwardNode();
        }
    }
}

void RRT::connectToBackNode()
{
    Node *qNearestFor = nearest(this->lastBackNode->position);
    Vector2f newPos = newConfig(this->lastBackNode, qNearestFor);
    while(!this->obstacles->isSegmentInObstacle(newPos, qNearestFor->position)){
        Node *qNewFor = new Node;
        qNewFor->position = newPos;
        add(qNearestFor, qNewFor);
        if(distance(this->lastBackNode->position, newPos) <= this->step_size)
            break;
        qNearestFor = qNewFor;
        newPos = newConfig(this->lastBackNode, qNearestFor);
    }
}

void RRT::connectToForwardNode()
{
    Node *qNearestBack = nearest(this->lastNode->position, this->backTree);
    Vector2f newPos = newConfig(this->lastNode, qNearestBack);
    while(!this->obstacles->isSegmentInObstacle(newPos, qNearestBack->position)){
        Node *qNewBack = new Node;
        qNewBack->position = newPos;
        addInBack(qNearestBack, qNewBack);
        if(distance(this->lastNode->position, newPos) <= this->step_size)
            break;
        qNearestBack = qNewBack;
        newPos = newConfig(this->lastNode, qNearestBack);
    }
}

void RRT::initBiConnect()
{
    this->tempNodes.clear();
    this->backTree.clear();

    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    lastNode = root;
    tempNodes.push_back(root);

    Node *rootBack = new Node;
    rootBack->parent = NULL;
    rootBack->position = endPos;
    lastBackNode =rootBack;
    backTree.push_back(rootBack);
}

bool RRT::checkReachGoal()
{
    if(distance(lastNode->position, lastBackNode->position) < END_DIST_THRESHOLD){
        return true;
    }
    return false;
}

void RRT::findPathBiConnect(bool success)
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
