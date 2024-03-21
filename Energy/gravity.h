//
// Created by Xiaopeng Zhang on 3/13/24.
//

#ifndef IPC2DSIM_GRAVITY_H
#define IPC2DSIM_GRAVITY_H
#include <vector>
#include "../eigen-3.4.0/Eigen/Dense"

#define GRAVITY 9.8f

float graEngVal(std::vector<Eigen::Vector2f>& pts, const std::vector<float>& masses)
{
    float res = 0.0f;
    Eigen::Vector2f gravity(0.0f, GRAVITY);
    for(int i = 0; i < pts.size(); i++)
    {
        res += pts[i].dot(gravity) * (-masses[i]);
    }
    return res;
}

std::vector<Eigen::Vector2f> graEngGrad(const std::vector<float>& masses)
{
    std::vector<Eigen::Vector2f> res;
    res.reserve(masses.size());
    for(auto mass : masses)
    {
        res.emplace_back(0.0f, GRAVITY * mass);
    }
    return res;
}




#endif //IPC2DSIM_GRAVITY_H
