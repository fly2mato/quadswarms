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
    PRM prm(N);

    cout << endl;
    for(uint i = 0; i < N; ++i) {
        for(uint j = 0; j < prm.edge[i].size(); ++j) {
            cout << prm.vertex[i].transpose() << " " << prm.vertex[prm.edge[i][j]].transpose() << endl;
        }
        // cout << prm.vertex[i].transpose() << " " << prm.vertex[i].transpose() << endl;
    }

    // Vector3d source(random(0,10), random(0,10), random(0,10));
    // Vector3d goal(random(90,100), random(90,100), random(90,100));

    // Vector3d source(random(0,10), random(0,10), 0);
    // Vector3d goal(random(90,100), random(90,100), 0);

    // Vector3d source(0,1,0);
    // Vector3d goal(1,3,0);


    // cout << "==================" << endl;
    // cout << source.transpose() << endl;
    // prm.SearchPathConstrain(source, goal);
    // cout << goal.transpose() << endl;

    // cout << "==================" << endl;
    // cout << source.transpose() << endl;
    // prm.SearchPath(source, goal);
    // cout << goal.transpose() << endl;

    vector<int> sources;
    vector<int> goals;

    // sources.push_back(0);
    // sources.push_back(4);

    // goals.push_back(5);
    // goals.push_back(1);

    for(int i = 0; i < 50; ++i) {
        sources.push_back(i);
        goals.push_back(i + 50);
    }
    vector< vector<int>> paths = prm.ComflictBasedSearch(sources, goals);
    


    // for(uint i = 0; i < paths.size(); ++i) {
    //     cout << "=========== " << i << "  ====" << endl;
    //     for(auto j = paths[i].begin(); j!=paths[i].end(); ++j) {
    //         // cout << *j << endl;
    //         cout << prm.vertex[*j].transpose() << endl;
    //     }
    //     cout << endl;
    // }

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
    // // int rgba[3] = {0x05, 0x19, 0x37};
    // // int rgbb[3] = {0xa8, 0xeb, 0x12};
    // int rgba[3] = {0x84, 0x5e, 0xc2};
    // int rgbb[3] = {0xf9, 0xf8, 0x71};

    // int step[3];
    // for(int i = 0; i < 3; ++i) {
    //     step[i] = ((rgbb[i] - rgba[i] + 255) % 255) / 5;
    //     cout << step[i] << endl;
    // }
    // for(int j = 0; j < 6; ++j) {
    //     for(int i = 0; i < 3; ++i) {
    //         cout << hex;
    //         cout << rgba[i] + j * step[i] << ',';
    //     }
    //     cout << endl;
    // }


    return 1;
}
