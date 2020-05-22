#ifndef RRTMULT_H
#define RRTMULT_H

#include "rrt.h"

class RRTMult
{
public:
    RRTMult();
    bool plan();
    void initialize();
    void getSimplePath();
    void seperateSimplePath();
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    Vector2f newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    void checkConnection(int *groupid, int *nodeid);
    void connectToGroup(int groupid, int nodeid);
    void findPath();

    // util functions
    int distance(Vector2f &p, Vector2f &q);
    void setStepSize(int step);
    void setMaxIterations(int iter);
    bool isSegInObstacle(Node *a, Node *b);

    Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2f startPos, endPos, goalPos;
    int max_iter;
    int step_size;
    int groupCount;
    bool success;

    vector<Node *> simplePath;
    vector<Node *> tempNodes;
    vector< vector<Node *> > nodeGroups;
};

#endif // RRTMULT_H
