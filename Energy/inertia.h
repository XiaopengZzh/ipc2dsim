//
// Created by Xiaopeng Zhang on 3/13/24.
//

#ifndef IPC2DSIM_INERTIA_H
#define IPC2DSIM_INERTIA_H
#include <vector>
#include "../eigen-3.4.0/Eigen/Dense"
#include "sparseMatrix.h"

float inertiaEngVal(std::vector<Eigen::Vector2f>& x,
                    std::vector<Eigen::Vector2f>& x_tilde,
                    const std::vector<float>& masses)
{
    float res = 0.0f;
    for(int i = 0;i < x.size(); i++)
    {
        Eigen::Vector2f diff = x[i] - x_tilde[i];
        res += 0.5f * masses[i] * diff.dot(diff);
    }
    return res;
}

std::vector<Eigen::Vector2f> inertiaEngGrad(std::vector<Eigen::Vector2f>& x,
                                            std::vector<Eigen::Vector2f>& x_tilde,
                                            const std::vector<float>& masses)
{
    std::vector<Eigen::Vector2f> res;
    res.reserve(masses.size());
    for(int i = 0; i < masses.size(); i++)
    {
        res.emplace_back(masses[i] * (x[i] - x_tilde[i]));
    }
    return res;
}

simsparseMat inertiaEngHessian(std::vector<Eigen::Vector2f>& x,
                               std::vector<Eigen::Vector2f>& x_tilde,
                               const std::vector<float>& masses)
{
    simsparseMat res;
    for(int i = 0; i < masses.size(); i++)
    {
        res.loc_i.push_back(2 * i);
        res.loc_i.push_back(2 * i + 1);
        res.loc_j.push_back(2 * i);
        res.loc_j.push_back(2 * i + 1);
        res.vals.push_back(masses[i]);
        res.vals.push_back(masses[i]);
    }
    return res;
}


#endif //IPC2DSIM_INERTIA_H
