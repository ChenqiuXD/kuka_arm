#include "rrt.h"

bool RRT::planBi()
{
    bool result = false;
    initBi();
    bool growBackTree = false;

    for(int i=0;i<this->max_iter;++i){
        Node *q = getRandomNode();
        if(q){
            Node *qNearest;
            if(growBackTree){
                qNearest = nearest(q->position, this->backTree);
            }else{
                qNearest = nearest(q->position, this->tempNodes);
            }

            Vector2f newPos = newConfig(q, qNearest);
            if(!this->obstacles->isSegmentInObstacle(newPos, qNearest->position)){
                Node *qNew = new Node;
                qNew->position = newPos;
                if(growBackTree){
                    addInBack(qNearest, qNew);
                    Node *qNearestFor = nearest(qNew->position, this->tempNodes);
                    newPos = newConfig(qNew, qNearestFor);
                    if(!this->obstacles->isSegmentInObstacle(newPos, qNearestFor->position)){
                        Node *qNewFor = new Node;
                        qNewFor->position = newPos;
                        add(qNearestFor, qNewFor);
                    }
                }
                else{
                    add(qNearest, qNew);
                    Node *qNearestBack = nearest(qNew->position, this->backTree);
                    newPos = newConfig(qNew, qNearestBack);
                    if(!this->obstacles->isSegmentInObstacle(newPos, qNearestBack->position)){
                        Node *qNewBack = new Node;
                        qNewBack->position = newPos;
                        addInBack(qNearestBack, qNewBack);
                    }
                }
                growBackTree = !growBackTree;
            }
        }
        if(checkReachGoalBi()){
            result = true;
            break;
        }
    }
    return result;
}

void RRT::initBi()
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

void RRT::addInBack(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNearest->children.push_back(qNew);
    backTree.push_back(qNew);
    lastBackNode = qNew;
}

bool RRT::checkReachGoalBi()
{
    // Node *nearestInFor = nearest(this->lastBackNode->position);
    if(distance(lastNode->position, lastBackNode->position) < END_DIST_THRESHOLD){
        return true;
    }
    return false;
}

void RRT::findPathBi(bool success)
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
