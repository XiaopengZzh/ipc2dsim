//
// Created by Xiaopeng Zhang on 3/14/24.
//

#ifndef IPC2DSIM_BARRIER_H
#define IPC2DSIM_BARRIER_H

#include <vector>
#include "../eigen-3.4.0/Eigen/Dense"
#include "sparseMatrix.h"
#include <cmath>
#include <algorithm>

float dhat = 0.01;
float kappa = 1e5;

float barrierEngVal(std::vector<Eigen::Vector2f>& x, float y_ground, std::vector<float>& contact_area)
{
    float res = 0.0f;
    for(int i = 0; i < x.size(); i++)
    {
        float d = x[i](1) - y_ground;
        if(d < dhat)
        {
            float s = d /dhat;
            res += contact_area[i] * dhat * kappa / 2 * (s - 1) * log(s);
        }
    }
    return res;
}

std::vector<Eigen::Vector2f> barrierEngGrad(std::vector<Eigen::Vector2f>& x,
                                            float y_ground,
                                            std::vector<float>& contact_area)
{
    std::vector<Eigen::Vector2f> res;
    res.reserve(x.size());
    for(int i = 0; i < x.size(); i++)
    {
        res.emplace_back(0.0f, 0.0f);
    }
    for(int i = 0; i < x.size(); i++)
    {
        float d = x[i](1) - y_ground;
        if(d < dhat)
        {
            float s = d / dhat;
            res[i](1) = contact_area[i] * dhat * (kappa / 2 * (log(s) / dhat + (s - 1) / d));
        }
    }
    return res;
}

simsparseMat barrierEngHess(std::vector<Eigen::Vector2f>& x, float y_ground, std::vector<float>& contact_area)
{
    simsparseMat res;
    for(int i = 0; i < x.size(); i++)
    {
        res.loc_i.push_back(0);
        res.loc_j.push_back(0);
        res.vals.push_back(0.0);
    }

    for(int i = 0; i < x.size(); i++)
    {
        res.loc_i[i] = 2 * i + 1;
        res.loc_j[i] = 2 * i + 1;
        float d = x[i](1) - y_ground;
        if(d < dhat)
        {
            res.vals[i] = contact_area[i] * dhat * kappa / (2 * d * d * dhat) * (d + dhat);
        }
        else
        {
            res.vals[i] = 0.0f;
        }
    }
    return res;
}

float init_step_size(std::vector<Eigen::Vector2f>& x, float y_ground, std::vector<Eigen::Vector2f>& p)
{
    float alpha = 1.0;
    for(int i = 0; i < x.size(); i++)
    {
        if(p[i](1) < 0.0f)
        {
            if(alpha > 0.9f * (y_ground - x[i](1)) / p[i](1))
            {
                alpha = 0.9f * (y_ground - x[i](1)) / p[i](1);
            }
        }
    }
    return alpha;
}


#endif //IPC2DSIM_BARRIER_H
