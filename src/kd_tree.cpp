#include <kd_tree.h>

void swap(int *i, int *j) {
    int temp = *i;
    *i = *j;
    *j = temp;
    return ;
}

KDTree::KDTree(vector<Vector3d> & p) 
    : points(p)  
    , min_dist(0)  
    , max_dist(-1)
{
    unsigned int n = points.size();
    parent = vector<int>(n, -1); 
    left_child = vector<int>(n, -1);
    right_child = vector<int>(n, -1);
    feature_id = vector<int>(n, -1);
    root_id = -1;

    vector<int> index(n);
    
    for(uint i = 0; i < n; ++i) {
        index[i] = i;
    }
    BuildTree(index, &root_id, 0, n, -1);

}

void KDTree::BuildTree(vector<int> & index, int * root, int left_p, int right_p, int parent_id){
    //right_p 等价 .end(), 最后一个元素的后一个位置
    if (left_p == right_p - 1) {
        *root = index[left_p];
        left_child[*root] = right_child[*root] = -1;
        parent[*root] = parent_id;
        feature_id[*root] = -1;
        return ;
    }

    int temp_fid;
    if ( parent_id == -1) {
        temp_fid = ChooseFeature(index, left_p, right_p);
    } else {
        temp_fid = (feature_id[parent_id] + 1) % 3;
    }

    int median_id = (right_p + left_p)/2;
    SplitPoints(index, left_p, right_p, temp_fid, median_id);

    *root = index[median_id];
    parent[*root] = parent_id;
    feature_id[*root] = temp_fid;

    if (left_p < median_id) {
        BuildTree(index, &left_child[*root], left_p, median_id, *root);
    }

    if (median_id + 1 < right_p) {
        BuildTree(index, &right_child[*root], median_id+1, right_p, *root);
    }
    return ;
}

int KDTree::ChooseFeature(vector<int> &index, int left_p, int right_p){
    Vector3d sum_sqr = Vector3d::Zero(3, 1);
    Vector3d sum = Vector3d::Zero(3, 1);

    for(int i = left_p; i < right_p; ++i) {
        sum += points[index[i]];
        sum_sqr += points[index[i]].cwiseProduct(points[index[i]]);
    }

    double n = right_p - left_p;
    int row, col;
    Vector3d variance = sum_sqr / n - (sum.cwiseProduct(sum) / n / n);
    variance.maxCoeff(&row, &col);
    return row;
}

void KDTree::SplitPoints(vector<int> &index, int left_p, int right_p, int f_id, int mid_p){
    int i,j,temp;
    i = temp = left_p;
    j = right_p;
    while(1) {
        while(++i < right_p && points[index[i]](f_id) < points[index[temp]](f_id)) {;}
        while(points[index[--j]](f_id) > points[index[temp]](f_id) && j > left_p) {;}
        if (i > j) break;
        swap(&index[i], &index[j]);
    }
    swap(&index[temp], &index[j]);
    if (j > mid_p && left_p < j) SplitPoints(index, left_p, j, f_id, mid_p);
    if (j < mid_p && i < right_p) SplitPoints(index, i, right_p, f_id, mid_p);
    return ;
}

int KDTree::GetBrother(int p) {
    if (p >= 0 && parent[p] >= 0) {
        if (left_child[parent[p]] == p) return right_child[parent[p]];
        else return left_child[parent[p]];
    }
    return -1;
}


int KDTree::SearchToLeaf(const Vector3d &target, int p){
    if (feature_id[p] < 0) return p;
    if (left_child[p] < 0 || (target(feature_id[p]) >= points[p](feature_id[p]) && right_child[p] >= 0)) {
        return SearchToLeaf(target, right_child[p]);
    }
    if (right_child[p] < 0 || (target(feature_id[p]) <= points[p](feature_id[p]) && left_child[p] >= 0)) {
        return SearchToLeaf(target, left_child[p]);
    }
    return p;
}


int KDTree::GetNearestNeighbor(const Vector3d & target, int cur_root) {
    if (cur_root < 0) cur_root = root_id;
    int cur = SearchToLeaf(target, cur_root);
    int bro = -1;
    double dist_cur, dist_bro;//, dist_best = INFINITY;
    // int best = cur;
    // int bro_best = -1;    

    while(1) {
        dist_cur = GetEulerDist(target, points[cur]);
        // if (dist_cur < dist_best) {
        //     dist_best = dist_cur;
        //     best = cur;
        // }
        if (dist_cur >= min_dist && (max_dist < 0 || dist_cur < max_dist)) {
            max_heap.Update(dist_cur, cur);
        }

        if (cur != cur_root) {
            bro = GetBrother(cur);
            // cout << "search1: " << cur << ',' << bro << endl;
            if (bro >= 0) {
                dist_bro = GetPlaneDist(target, points[parent[cur]], feature_id[parent[cur]]);
                // cout << "search: " << dist_bro << endl;
                // if (dist_bro < dist_best) {
                if (dist_bro < max_heap.array[0]) {
                    // bro_best = GetNearestNeighbor(target, bro);
                    // dist_bro = GetEulerDist(target, points[bro_best]);
                    // if (dist_bro < dist_best) {
                    //     best = bro_best;
                    //     dist_best = dist_bro;
                    // }
                    GetNearestNeighbor(target, bro);
                }
            }
            cur = parent[cur];
        } else {
            break;
        }    
    }
    return 1;   
}


vector<int> KDTree::GetKNN(const Vector3d & target, int K, double mindist, double maxdist) {
    vector<double> dist;
    dist.clear();
    min_dist = mindist;
    max_dist = maxdist;
    for(int i = 0; i < K; ++i) {
        double xdist = (target - points[i]).norm();
        if (xdist < min_dist || (xdist > max_dist && max_dist >= 0)) dist.push_back(-1);        
        else dist.push_back(xdist);
    }

    max_heap = MaxHeap();
    max_heap.MaxHeapInit(K, dist);
    max_heap.HeapSort();
    GetNearestNeighbor(target, -1);
    return max_heap.id;
}

void MaxHeap::HeapSwap(int a, int b){
    double temp = array[a];
    array[a] = array[b];
    array[b] = temp;

    int temp_id = id[a];
    id[a] = id[b];
    id[b] = temp_id;
}

void MaxHeap::MaxHeapify(int index, int len) {
    int li = (index << 1) + 1;
    int ri = li + 1;
    int cMax = li;

    if(li > len) return;               // left child is out of range.               
    if(ri <= len && array[ri] < array[li]) // choose the bigger one.  
        cMax = ri;              
    if(array[cMax] < array[index]){        // if  
        HeapSwap(index, cMax);        // swapped,   
        MaxHeapify(cMax, len);    // next check should be executed.  
    }
}

void MaxHeap::HeapSort() {
    int len = Num - 1;
    int lastNodeIndex = (len - 1) >> 1;
    for(int i = lastNodeIndex; i >= 0; i--){ 
        MaxHeapify(i, len);
    }

    for(int i = 0; i < len; ++i) {
        HeapSwap(0, len-i);
        MaxHeapify(0, len-i-1);
    }    
}

void MaxHeap::MaxHeapInit(int N, vector<double> &p){
    Num = N;
    array.clear(); 
    id.clear(); 
    for(int i = 0; i < N; ++i) {
        if (p[i] < 0) {
            array.push_back(INFINITY);
            id.push_back(-1);
        } else {
            array.push_back(p[i]);
            id.push_back(i);
        }
    }
}

void MaxHeap::Update(double value, int new_id){
    if (value < array[0] && new_id >= Num) {
        array[0] = value;
        id[0] = new_id;
        HeapSort();
    }    
}










// Node * GetBrother(Node * p) {
//     if (p && p->parent) {
//         if (p->parent->left_child == p) return p->parent->right_child;
//         else return p->parent->left_child;
//     }
//     return NULL;
// }


// void KDTree::DeleteTree(Node *p){
//     if (p->left_child) DeleteTree(p->left_child);
//     if (p->right_child) DeleteTree(p->right_child);
//     delete p;
// }

// void swap(vector<Vector3d> & points, int i, int j) {
//     Vector3d temp = points[i];
//     points[i] = points[j];
//     points[j] = temp;
//     return ;
// }

// void KDTree::BuildTree(vector<Vector3d> & points, Node * node, int left_p, int right_p){
//     //right_p 等价 .end(), 最后一个元素的后一个位置
//     if (left_p == right_p - 1) {
//         node->data = points[left_p];
//         node->left_child = node->right_child = NULL;
//         node->feature_id = -1;
//         return ;
//     }

//     if (node == root) {
//         node->feature_id = ChooseFeature(points, left_p, right_p);
//     } else {
//         node->feature_id = (node->parent->feature_id + 1) % 3;
//     }

//     int median_id = (right_p + left_p)/2;
//     SplitPoints(points, left_p, right_p, node->feature_id, median_id);
//     node->data = points[median_id];
    
//     if (left_p < median_id) {
//         node->left_child = new Node;
//         node->left_child->parent = node;
//         BuildTree(points, node->left_child, left_p, median_id);
//     }

//     if (median_id + 1 < right_p) {
//         node->right_child = new Node;
//         node->right_child->parent = node;
//         BuildTree(points, node->right_child, median_id+1, right_p);
//     }
//     return ;
// }

// int KDTree::ChooseFeature(vector<Vector3d> &points, int left_p, int right_p){
//     Vector3d sum_sqr = Vector3d::Zero(3, 1);
//     Vector3d sum = Vector3d::Zero(3, 1);

//     for(int i = left_p; i < right_p; ++i) {
//         sum += points[i];
//         sum_sqr += points[i].cwiseProduct(points[i]);
//     }
//     double n = right_p - left_p;
//     int row, col;
//     Vector3d variance = sum_sqr / n - (sum.cwiseProduct(sum) / n / n);
//     variance.maxCoeff(&row, &col);
//     return row;
// }

// void KDTree::SplitPoints(vector<Vector3d> &points, int left_p, int right_p, int f_id, int mid_p){
//     int i,j,temp;
//     i = temp = left_p;
//     j = right_p;
//     while(1) {
//         while(i < right_p && points[++i](f_id) < points[temp](f_id)) {;}
//         while(points[--j](f_id) > points[temp](f_id) && j > left_p) {;}
//         if (i > j) break;
//         swap(points, i, j);
//     }
//     swap(points, temp, j);
//     if (j > mid_p && left_p < j) SplitPoints(points, left_p, j, f_id, mid_p);
//     if (j < mid_p && i < right_p) SplitPoints(points, i, right_p, f_id, mid_p);
//     return ;
// }


// Node* KDTree::SearchToLeaf(Vector3d target, Node* p){
//     if (p->feature_id < 0) return p;
//     if (target(p->feature_id) >= p->data(p->feature_id) && p->right_child) {
//         return SearchToLeaf(target, p->right_child);
//     }
//     if (target(p->feature_id) <= p->data(p->feature_id) && p->left_child) {
        
//         return SearchToLeaf(target, p->left_child);
//     }
//     return p;
// }


// Node* KDTree::GetNearestNeighbor(Vector3d & nearest_neighbor, const Vector3d & target, Node* cur_root = NULL) {
//     if (!cur_root) cur_root = root;

//     Node *cur = SearchToLeaf(target, cur_root);
//     Node *best = cur;
//     Node *bro = NULL;
//     Node *bro_best = NULL;    
//     double dist_cur, dist_bro, dist_best = INFINITY;

//     while(1) {
//         dist_cur = GetEulerDist(target, cur->data);
//         if (dist_cur < dist_best) {
//             dist_best = dist_cur;
//             best = cur;
//         }

//         if (cur != cur_root) {
//             bro = GetBrother(cur);
//             if (bro) {
//                 dist_bro = GetPlaneDist(target, cur->parent->data, cur->parent->feature_id);
//                 if (dist_bro < dist_best) {
//                     bro_best = GetNearestNeighbor(nearest_neighbor, target, bro);
//                     dist_bro = GetEulerDist(target, bro_best->data);
//                     if (dist_bro < dist_best) {
//                         best = bro_best;
//                         dist_best = dist_bro;
//                     }
//                 }
//             }
//             cur = cur->parent;
//         } else {
//             break;
//         }    
//     }

//     nearest_neighbor = best->data;
//     return best;   
// }
