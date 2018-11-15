#ifndef PRM_H
#define PRM_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <list>
#include <ctime>
#include <cstdlib>
#include <kd_tree.h>
#include <stack>

using namespace std;
using namespace Eigen;

struct Node{
    Vector3d point;
    vector<int> next;
};

class PRM {
private:
    Vector3d map_size;
    int nearest_num;
    double nearest_radius;
    double neighbor_min_dist;

public:
    vector<Vector3d> vertex;
    vector< vector<int>> edge;
    vector< vector<double>> edge_dist;
    // vector<Node> vertex;

    KDTree *p_kd_tree;

    // PRM(){};
    ~PRM(){delete p_kd_tree;};
    PRM(unsigned int sampleN); 

    vector<int> SearchPath(Vector3d source, Vector3d goal);
    int AstarSearch(int source_index, int goal_index, vector<int> &path);
    double HeuristicCostEstimate(int start, int goal);

    double random(double start, double end);
};


#endif
