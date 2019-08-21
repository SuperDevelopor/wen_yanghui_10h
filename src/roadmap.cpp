#include "roadmap.h"

RoadMap::RoadMap()
{
    Node nd;
    // adjacent_matrix_.resize(10 * 10);
    vector<Node> t;
    for (int i = 0; i < 10; i++) {
        t.clear();
        for (int j = 0; j < 10; j++) {
            nd.x = i;
            nd.y = j;
            t.push_back(nd);
        }
        scatter_map_.push_back(t);
        // adjacent_matrix_[i].resize(10 * 10);
    }

    int k = 0;
    for (int i = 0; i < scatter_map_.size(); i++) {
        for (int j = 0; j < 10; j++) {
            k = i - 1;
            if (k >= 0 && k <= 10) {
                scatter_map_[i][j].adjacent.push_back(&scatter_map_[k][j]);
            }

            k = i + 1;
            if (k >= 0 && k <= 10) {
                scatter_map_[i][j].adjacent.push_back(&scatter_map_[k][j]);
            }

            k = j - 1;
            if (k >= 0 && k <= 10) {
                scatter_map_[i][j].adjacent.push_back(&scatter_map_[i][k]);
            }

            k = j + 1;
            if (k >= 0 && k <= 10) {
                scatter_map_[i][j].adjacent.push_back(&scatter_map_[i][k]);
            }
        }
    }
}
