//
// Created by Xiaopeng Zhang on 3/14/24.
//

#ifndef IPC2DSIM_SPRING_H
#define IPC2DSIM_SPRING_H

#include <vector>
#include "../eigen-3.4.0/Eigen/Dense"
#include "sparseMatrix.h"

float springEngVal(std::vector<Eigen::Vector2f>& x,
                   std::vector<Eigen::Vector2i>& edges,
                   std::vector<float>& l_sq,
                   std::vector<float>& k)
{
    float res = 0.0f;
    for(int i = 0; i < edges.size(); i++)
    {
        Eigen::Vector2f diff = x[edges[i][0]] - x[edges[i][1]];
        float temp = diff.dot(diff) - l_sq[i] - 1;
        res += l_sq[i] * 0.5f * k[i] * temp * temp;
    }
    return res;
}

std::vector<Eigen::Vector2f> springEngGrad(std::vector<Eigen::Vector2f>& x,
                                           std::vector<Eigen::Vector2i>& edges,
                                           std::vector<float>& l_sq,
                                           std::vector<float>& k)
{
    std::vector<Eigen::Vector2f> res;
    res.reserve(x.size());
    for(int i = 0; i < x.size(); i++)
    {
        res.emplace_back(0.0f, 0.0f);
    }

    for(int i = 0; i < edges.size(); i++)
    {
        Eigen::Vector2f diff = x[edges[i][0]] - x[edges[i][1]];
        Eigen::Vector2f g_diff = 2 * k[i] * (diff.dot(diff) / l_sq[i] - 1) * diff;
        res[edges[i][0]] += g_diff;
        res[edges[i][1]] -= g_diff;
    }
    return res;
}

simsparseMat springEngHess(std::vector<Eigen::Vector2f>& x,
                           std::vector<Eigen::Vector2i>& edges,
                           std::vector<float>& l_sq,
                           std::vector<float>& k)
{
    simsparseMat res;
    for(int i = 0; i < 16 * edges.size(); i++)
    {
        res.loc_i.push_back(0);
        res.loc_j.push_back(0);
        res.vals.push_back(0.0f);
    }
    for(int i = 0; i < edges.size(); i++)
    {
        Eigen::Vector2f diff = x[edges[i][0]] - x[edges[i][1]];
        Eigen::Matrix2f H_diff = 2 * k[i] / l_sq[i] *
                (2 * diff * diff.transpose() + (diff.dot(diff) - l_sq[i]) *  Eigen::Matrix2f::Identity());
        Eigen::Matrix4f temp;
        temp.topLeftCorner(2, 2) = H_diff;
        temp.topRightCorner(2, 2) = -H_diff;
        temp.bottomLeftCorner(2, 2) = - H_diff;
        temp.bottomRightCorner(2, 2) = H_diff;
        Eigen::Matrix4f H_local = mySPDProjection(temp);

        for(int nI = 0; nI < 2; nI++)
        {
            for(int nJ = 0; nJ < 2; nJ++)
            {
                int indStart = i * 16 + (nI * 2 + nJ) * 4;
                for(int r = 0; r < 2; r++)
                {
                    for(int c = 0; c < 2; c++)
                    {
                        res.loc_i[indStart + r * 2 + c] = edges[i][nI] * 2 + r;
                        res.loc_j[indStart + r * 2 + c] = edges[i][nJ] * 2 + c;
                        res.vals[indStart + r * 2 + c] = H_local(nI * 2 + r, nJ * 2 + c);
                    }
                }
            }
        }

    }
    return res;
}





#endif //IPC2DSIM_SPRING_H
