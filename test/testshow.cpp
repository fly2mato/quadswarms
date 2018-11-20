#include <PRM.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <fstream>

using namespace std;
using namespace Eigen;

double random(double start, double end)
{
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}

int main(int argc, char *argv[]) {
    uint N = 10;
    if (argc > 1) {
        N = atoi(argv[1]);
    }
    cout << "n = " << N << endl;
    
    srand(unsigned(time(0)));
    
    ifstream fid("../test/points.txt");

    int paths_num;
    int fig_num;
    fid >> paths_num >> fig_num;

    vector<Vector3d> sources;
    vector<Vector3d> goals;
    double x,y,z;
    sources.clear();

    int nn = 10;
    for(uint i = 0; i < paths_num; ++i) {
        fid >> x >> y >> z;
        if (i >= nn) continue;
        sources.push_back(Vector3d(x,y,z));
    }
    goals.clear();
    for(uint i = 0; i < paths_num; ++i) {
        fid >> x >> y >> z;
        if (i >= nn) continue;
        goals.push_back(Vector3d(x,y,z));
    }
    fid.close();

    PRM prm(N, sources, goals, Vector3d(0,-50,0), Vector3d(100,50,100));

    // cout << endl;
    // for(uint i = 0; i < N; ++i) {
    //     for(uint j = 0; j < prm.edge[i].size(); ++j) {
    //         cout << prm.vertex[i].transpose() << " " << prm.vertex[prm.edge[i][j]].transpose() << endl;
    //     }
    //     // cout << prm.vertex[i].transpose() << " " << prm.vertex[i].transpose() << endl;
    // }

    vector<int> sources_index;
    vector<int> goals_index;

    for(int i = 0; i < paths_num; ++i) {
        if (i >= nn) continue;
        sources_index.push_back(i);
        // goals_index.push_back(i + paths_num);
        goals_index.push_back(i + nn);
    }
    vector< vector<int>> paths = prm.ComflictBasedSearch(sources_index, goals_index);
    
    cout << paths_num << endl;

    ofstream fout("output.txt");
    for(uint i = 0; i < paths.size(); ++i) {
        fout << "#" << endl;
        for(auto j = paths[i].begin(); j!=paths[i].end(); ++j) {
            fout << prm.vertex[*j](0) << ',' << 
                    prm.vertex[*j](1) << ',' << 
                    prm.vertex[*j](2) <<  
                    endl;
        }
    }
    fout.close();

    return 1;
}
