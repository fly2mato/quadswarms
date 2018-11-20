#ifndef SEARCH_TREE_H
#define SEARCH_TREE_H

#include<vector>

using namespace std;


struct CBS_Node {
    int update_path_id;
    vector< vector<int>> paths;
    vector< vector<int>> constrains_path_id; 
    vector< vector<int>> constrains_path_step;
};







#endif