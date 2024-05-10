//
// Created by Xiaopeng Zhang on 3/13/24.
//

#ifndef IPC2DSIM_UTILS_H
#define IPC2DSIM_UTILS_H

#include <vector>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <iostream>

/*
using TArrayEdges = std::vector<std::pair<unsigned int, unsigned int>>;

void append(TArrayEdges& edges, const TArrayEdges& edgesToAdd)
{
    unsigned int baseNum = edges.size();
    for(auto edge : edgesToAdd)
    {
        edges.emplace_back(edge.first + baseNum, edge.second + baseNum);
    }
}
*/

void ensure_directory_exists(const std::string& path)
{
    struct stat st;
    if(stat(path.c_str(), &st) != 0)
    {
        if(mkdir(path.c_str(), 0755) != 0)
        {
            std::cerr << "Failed to create directory : " << path << std::endl;
        }
    }
}




#endif //IPC2DSIM_UTILS_H
