//
// Created by Xiaopeng Zhang on 3/23/24.
//

#ifndef IPC2DSIM_PHYSCENE_H
#define IPC2DSIM_PHYSCENE_H

#include "eigen-3.4.0/Eigen/Core"


class phyScene
{
public:
    phyScene() = default;

    std::vector<Eigen::Vector2f> vertices;
    std::vector<std::pair<unsigned int, unsigned int>> edges;

    //void oneTimestepImpl();

};




#endif //IPC2DSIM_PHYSCENE_H
