#include <PRM.h>

double random(double,double);



PRM::PRM(unsigned int sampleN) 
    : map_size(Vector3d(100,100,100))
    , nearest_num(5)
    , nearest_radius(2)
    , neighbor_min_dist(1.9)
{
    vertex.clear();
    vertex.reserve(sampleN);

    edge = vector< vector<int>>(sampleN);
    edge_dist = vector< vector<double>>(sampleN);
    
    srand(unsigned(time(0)));

    // Vector3d temp;
    // for(int i = 0; i < sampleN; ++i) {
    //     temp = Vector3d(random(0, map_size(0)), random(0, map_size(1)), random(0, map_size(2)));
    //     vertex.push_back(temp);                
    // }


    Vector3d temp;
    long int count = 0;
    while (!(count > (long int)(sampleN * sampleN) || vertex.size() == sampleN)) {
        count ++;
        temp = Vector3d(random(0, map_size(0)), random(0, map_size(1)), random(0, map_size(2)));
        int flag = 0;
        for(unsigned int j = 0; j < vertex.size(); ++j) {
            double di = (temp - vertex[j]).norm();
            if (di < nearest_radius) {
                flag = 1;
                break;
            }
        }
        if (flag) continue;
        vertex.push_back(temp);                
    }
    
    sampleN = vertex.size();
    cout << "sampleN : " << sampleN << endl;
    p_kd_tree = new KDTree(vertex);

    vector<int> ans;
    
    for(uint i = 0; i < sampleN; ++i) {
        ans = p_kd_tree->GetKNN(vertex[i], 4, 2, -1);
        for(uint j = 0; j < 4; ++j) {
            if (ans[j] == i || ans[j] == -1) continue;
            int flag = 0;
            for(unsigned int k = 0; k < edge[i].size(); ++k) {
                if (edge[i][k] == ans[j]) {
                    flag = 1;
                    break;
                }
            }
            if (flag == 0) {
                edge[i].push_back(ans[j]);
                edge_dist[i].push_back((vertex[i] - vertex[ans[j]]).norm());
            }

            flag = 0;
            for(uint k = 0; k < edge[ans[j]].size(); ++k) {
                if (edge[ans[j]][k] == i) {
                    flag = 1;
                    break;
                }
            }
            if (flag == 0) {
                edge[ans[j]].push_back(i);
                edge_dist[ans[j]].push_back((vertex[i] - vertex[ans[j]]).norm());
            }
        }
    }
}

vector<int> PRM::SearchPath(Vector3d source, Vector3d goal){
    vector<int> source_index;
    vector<int> goal_index;

    source_index = p_kd_tree->GetKNN(source, 1, 0, -1);
    goal_index = p_kd_tree->GetKNN(goal, 1, 0, -1);

    vector<int> path;

    if (AstarSearch(source_index[0], goal_index[0], path)) {
        for(uint i = 0; i < path.size(); ++i) {
            cout << vertex[path[i]].transpose()<<endl;
        }
    }
    return path;
}


int PRM::AstarSearch(int source, int goal, vector<int> &path){

    vector<double> gScore(vertex.size(), 10000);
    gScore[source] = 0;
    
    vector<double> fScore(vertex.size(), 10000);
    fScore[source] = HeuristicCostEstimate(source, goal);
    
    vector<int> cameFrom(vertex.size(), -1);

    list<int> openSet; 
    vector<int> closedSet; 

    openSet.clear();
    openSet.push_back(source);
    closedSet.clear();
    cameFrom.clear();

    path.push_back(source);

    int search_complete_flag = 0;

    while(openSet.size() > 0) {
        list<int>::iterator i, mini;
        i = mini = openSet.begin();
        int ni = *i;
        for(; i != openSet.end(); ++i) {
            if (fScore[*i] < fScore[*mini]) {
                mini = i; 
                ni = *i;   
            }
        }

        if (ni == goal) {
            search_complete_flag = 1;
            stack<int> GoTo;
            while(ni > 0) {
                GoTo.push(ni);      
                ni = cameFrom[ni];
            }
            GoTo.pop(); //弹出起始网格点
            while(!GoTo.empty() && GoTo.size() > 1) {
                ni = GoTo.top();
                path.push_back(ni);
                GoTo.pop();
            }
            path.push_back(goal);
            break;
        }

        closedSet.push_back(ni);
        if (mini!= openSet.end()) 
            openSet.erase(mini);

        for(uint xx = 0; xx < edge[ni].size(); ++xx){

            int xi = edge[ni][xx];
            
            // if (IsGridOccupy(neighbor.row(xx))) {
            //     cameFrom[xi] = -1;
            //     gScore[xi] = 10000;
            //     fScore[xi] = 10000;
            //     continue;
            // }

            if (find(closedSet.begin(), closedSet.end(), xi) != closedSet.end()){
                continue;
            }

            if (find(openSet.begin(), openSet.end(), xi) == openSet.end()) {
                openSet.push_back(xi);
            }

            double tentative_gScore = gScore[ni] + edge_dist[ni][xx];
            //增加航点间的转向代价，distAngle函数计算两向量夹角(rad).
            /*double tentative_gScore;
            //cout << "close set size::" << closedSet.size() << endl;
            if (closedSet.size() > 1){
                Vector3d p_ni, p_ni_prev, vec_a, vec_b;
                p_ni = SetIndex(ni).cast<double>().cwiseProduct(unit);
                p_ni_prev = SetIndex(*(closedSet.end() - 2)).cast<double>().cwiseProduct(unit);
                vec_a = p_ni - p_ni_prev;
                //前后两线段夹角.
                vec_b = SetIndex(xi).cast<double>().cwiseProduct(unit)- p_ni;
                //起点到终点的直线方向代价最小.
                //vec_b = SetIndex(xi).cast<double>().cwiseProduct(unit);
                tentative_gScore = gScore[ni] + dist(xx) + 1 * distAngle(vec_a, vec_b);
            }
            else
                tentative_gScore = gScore[ni] + dist(xx);*/

            if (tentative_gScore >= gScore[xi])
                continue;
            
            cameFrom[xi] = ni;
            gScore[xi] = tentative_gScore;
            fScore[xi] = gScore[xi] + HeuristicCostEstimate(ni, goal);
        }            
    }  

    return search_complete_flag;
}


double PRM::HeuristicCostEstimate(int start, int goal){
    Vector3d d = (vertex[goal] - vertex[start]);
    return abs(d(0)) + abs(d(1)) + abs(d(2));
}



double PRM::random(double start, double end) {
    return start+(end-start)*rand()/(RAND_MAX + 1.0);
}