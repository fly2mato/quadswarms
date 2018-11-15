#include <PRM.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cstdlib>
#include <ctime>

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
    
    PRM prm(N);

    cout << endl;
    for(uint i = 0; i < N; ++i) {
        for(uint j = 0; j < prm.edge[i].size(); ++j) {
            cout << prm.vertex[i].transpose() << " " << prm.vertex[prm.edge[i][j]].transpose() << endl;
        }
        // cout << prm.vertex[i].transpose() << " " << prm.vertex[i].transpose() << endl;
    }

    srand(unsigned(time(0)));
    Vector3d source(random(0,10), random(0,10), random(0,10));
    Vector3d goal(random(90,100), random(90,100), random(90,100));

    // Vector3d source(random(0,10), random(0,10), 0);
    // Vector3d goal(random(90,100), random(90,100), 0);

    cout << "==================" << endl;
    cout << source.transpose() << endl;
    prm.SearchPath(source, goal);
    cout << goal.transpose() << endl;

    return 1;
}
