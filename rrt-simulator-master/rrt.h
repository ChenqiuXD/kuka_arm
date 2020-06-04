#ifndef RRT_H
#define RRT_H

#include "obstacles.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace Eigen;

struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
};

class RRT
{
public:
    RRT();
    bool plan();
    void initialize();
    void findPath(bool result);
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    int distance(Vector2f &p, Vector2f &q);
    Vector2f newConfig(Node *q, Node *qNearest);
    void add(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(int step);
    void setMaxIterations(int iter);
    void deleteNodes(Node *root);

    Obstacles *obstacles;
    vector<Node *> nodes;
    vector<Node *> path;
    Node *root, *lastNode;
    Vector2f startPos, endPos, goalPos;
    int max_iter;
    int step_size;

// -------------------------------------------------------------------------
// Following code is written as a modified version of RRT as a demonstration
// -------------------------------------------------------------------------
    bool planConnect();
    void initConnect();
    void getSimplePlan();
    void getBlockedNodes();
    bool isSegInObstacle(Node *a, Node *b);
    void findPathConnect(bool result);
    void setInitNode(Node *node);
    void setEndNode(Vector2f point);

    vector<Node *> simplePath;
    vector<Node *> tempNodes;   // Store the nodes calculating while connecting two blocked nodes
    vector<Node *> blockedNodes;

// -------------------------------------------------------------------------
// Bi-directional RRT
// -------------------------------------------------------------------------
    bool planBi();
    void initBi();
    Node* nearest(Vector2f point, vector<Node*> tree);
    void addInBack(Node *qNearest, Node *qNew);
    bool checkReachGoalBi();
    void findPathBi(bool);

    vector<Node *> backTree;
    Node *lastBackNode;

// -------------------------------------------------------------------------
// RRT-connect
// -------------------------------------------------------------------------
    bool planBiConnect();
    void initBiConnect();
    void findPathBiConnect(bool success);
    void connectToBackNode();
    void connectToForwardNode();
    void growBackTree(Node*);
    void growForwardTree(Node*);
    bool checkReachGoal();

// -------------------------------------------------------------------------
// RRT-MultConnect
// -------------------------------------------------------------------------
    bool planMultConnect();
    void initMultConnect();
    void findPathMultConnect(bool);
    bool getSimplePath();
    void getForPath();
    void getBackPath();

    bool success;
};

#endif // RRT_H
