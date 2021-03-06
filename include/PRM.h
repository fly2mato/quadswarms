#ifndef PRM_H
#define PRM_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>
#include <list>
#include <ctime>
#include <cstdlib>
#include <stack>
#include <queue>
#include <kd_tree.h>
#include <SearchTree.h>

#define MAX_PATH_STEP 100

// #define PRM_LOG

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
    PRM(uint sampleN, vector<Vector3d> &source, vector<Vector3d> &goals, Vector3d low_bound, Vector3d up_bound);

    vector<int> SearchPath(Vector3d source, Vector3d goal);
    vector<int> SearchPathConstrain(Vector3d source, Vector3d goal);

    vector< vector<int>> ComflictBasedSearch(vector<int> sources, vector<int> goals); //要求起、止点都已经作为节点加入roadmap中
    int IsPathsComflict(const vector< vector<int>> & paths, uint &cf_step_a, uint &cf_step_b,
                         uint &cf_a, uint &cf_b, 
                         uint &cf_id_a, uint &cf_id_b);


    int AstarSearch(int source_index, int goal_index, vector<int> &path);
    int ConstrainAstartSearch(int source_index, int goal_index, vector<int> &path,
                                vector<int> &constrain_id, vector<int>&constrain_step);

    double HeuristicCostEstimate(int start, int goal);
    double WattingTimeCost(int delta_id);

    double random(double start, double end);
};


#endif
