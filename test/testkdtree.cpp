#include <kd_tree.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace Eigen;

int main(int argc, char * argv[]) {
    int n = 100;
    int range = 1;
    if (argc > 1) {
        n = atoi(argv[1]);
    }
    
    vector<Vector3d> points;
    points.clear();
    
    double random(double,double);
    srand(unsigned(time(0)));
    for(int icnt = 0; icnt != n; ++icnt) {
        points.push_back(Vector3d(random(0, range), random(0, range), random(0,range)));
    }

    Vector3d nn;
    Vector3d target;

    KDTree kdtree(points); 


    vector<double> ddist(n);
    // int ans2[KNN_NUM];
    // double best = INFINITY;
    // double dist;
    // int best_index;

    for(int k = 0; k < n; ++k) {
        target = Vector3d(random(0, range), random(0, range), random(0,range));

        vector<int> ans = kdtree.GetKNN(target, 5, 0, -1);



        // for(int i = 0; i < n; ++i) {
        //     ddist[i] = (points[i]-target).norm();
        // }
        // for(int i = 0; i < KNN_NUM; ++i) {
        //     best = INFINITY;
        //     best_index = -1;
        //     for(int j = 0; j < n; ++j) {
        //         if (ddist[j] < best && (i == 0 || ddist[j] > ddist[ans2[i-1]])) {
        //             best = ddist[j];
        //             best_index = j;
        //         }
        //     }

        //     ans2[i] = best_index;            
        // }

        // for(int j = 0; j < KNN_NUM; ++j) {
        //     if (ans[j] != ans2[KNN_NUM - 1 - j]) {
        //         cout << "wrong!" << endl;
        //         cout << "target: " << target.transpose() << endl;

        //         for(int m = 0; m < KNN_NUM; ++m){   
        //             cout << ans[m] << ',' << ans2[KNN_NUM - 1 - m] << " : " << ddist[ans[m]] << ',' << ddist[ans2[KNN_NUM - 1 - m]] << endl;
        //         }
        //         cout << ddist[0] << endl;

        //         break;
        //     }
        // }
        
    }

    return 1;
}

double random(double start, double end)
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}
