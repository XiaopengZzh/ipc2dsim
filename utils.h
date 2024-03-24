//
// Created by Xiaopeng Zhang on 3/13/24.
//

#ifndef IPC2DSIM_UTILS_H
#define IPC2DSIM_UTILS_H

#include <vector>


using TArrayEdges = std::vector<std::pair<unsigned int, unsigned int>>;

void append(TArrayEdges& edges, const TArrayEdges& edgesToAdd)
{
    unsigned int baseNum = edges.size();
    for(auto edge : edgesToAdd)
    {
        edges.emplace_back(edge.first + baseNum, edge.second + baseNum);
    }
}





#endif //IPC2DSIM_UTILS_H
