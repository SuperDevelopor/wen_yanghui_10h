#include "roadmap.h"

RoadMap::RoadMap()
{
   //初始化地图
    Node nd;
    // adjacent_matrix_.resize(10 * 10);
    vector<Node> t;
    for (int i = 0; i < SQURE; i++) {
        t.clear();
        for (int j = 0; j < SQURE; j++) {
            nd.x = i;
            nd.y = j;
            t.push_back(nd);
        }
        scatter_map_.emplace_back(t);
        // adjacent_matrix_[i].resize(10 * 10);
    }

    int k = 0;
    for (int i = 0; i < SQURE; i++) {
        for (int j = 0; j < SQURE; j++) {
            k = i - 1;
            if (k >= 0 && k < SQURE) {
                scatter_map_[i][j].adjacent.emplace_back(&scatter_map_[k][j]);
            }

            k = i + 1;
            if (k >= 0 && k < SQURE) {
                scatter_map_[i][j].adjacent.emplace_back(&scatter_map_[k][j]);
            }

            k = j - 1;
            if (k >= 0 && k < SQURE) {
                scatter_map_[i][j].adjacent.emplace_back(&scatter_map_[i][k]);
            }

            k = j + 1;
            if (k >= 0 && k < SQURE) {
                scatter_map_[i][j].adjacent.emplace_back(&scatter_map_[i][k]);
            }
        }
    }
}
