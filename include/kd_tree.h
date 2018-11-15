#ifndef KD_TREE_H
#define KD_TREE_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cmath>

#define KNN_NUM 5


using namespace std;
using namespace Eigen;


class MaxHeap {
public:
    vector<double> array;
    vector<int> id;
    int Num;
    
    MaxHeap(){};
    ~MaxHeap(){};

    void MaxHeapInit(int N, vector<double> &p);
    void MaxHeapify(int index, int len);
    void HeapSwap(int a, int b);
    void HeapSort();
    void Update(double value, int id);

};





class KDTree {
private:
    vector<Vector3d> & points;
    vector<int> parent;
    vector<int> left_child;
    vector<int> right_child;
    vector<int> feature_id;
    int root_id;
    double min_dist;
    double max_dist;
    
    MaxHeap max_heap;

    // Node * root;

public:
    KDTree(vector<Vector3d> & points);
    ~KDTree(){};

    void BuildTree(vector<int> & index, int * root, int left_p, int right_p, int parent_id);
    int ChooseFeature(vector<int> &index, int left_p, int right_p);
    void SplitPoints(vector<int> &index, int left_p, int right_p, int f_id, int mid_p);

    int GetBrother(int p);
    int GetNearestNeighbor(const Vector3d & target, int cur_root);
    int SearchToLeaf(const Vector3d &target, int p);

    double GetEulerDist(const Vector3d &a, const Vector3d &b){return (a-b).norm();}
    double GetPlaneDist(const Vector3d &target, const Vector3d &node, int f_id) {return fabs(target(f_id) - node(f_id));}

    vector<int> GetKNN(const Vector3d & target, int K, double min_dist, double max_dist);



    // ~KDTree(){DeleteTree(root);}

    // void DeleteTree(Node *p);
    // void BuildTree(vector<Vector3d> &points, Node * root, int left_p, int right_p);
    // int ChooseFeature(vector<Vector3d> &points, int left_p, int right_p);
    // void SplitPoints(vector<Vector3d> &points, int left_p, int right_p, int f_id, int mid_p);

    // Node* GetNearestNeighbor(Vector3d & nearest_neighbor, const Vector3d & target, Node* cur_root);
    // Node* SearchToLeaf(Vector3d target, Node* p);
    // double GetEulerDist(const Vector3d &a, const Vector3d &b){return (a-b).norm();}
    // double GetPlaneDist(const Vector3d &target, const Vector3d &node, int f_id) {return fabs(target(f_id) - node(f_id));}
};


#endif
