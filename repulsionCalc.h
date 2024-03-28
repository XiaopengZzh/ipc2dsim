//
// Created by Xiaopeng Zhang on 3/27/24.
//

#ifndef IPC2DSIM_REPULSIONCALC_H
#define IPC2DSIM_REPULSIONCALC_H

#include "eigen-3.4.0/Eigen/Core"

enum class EPointEdgeDistanceType
{
    P_E0,
    P_E1,
    P_E,
    AUTO
};

EPointEdgeDistanceType point_edge_dist_type(const Eigen::Vector2f& p, const Eigen::Vector2f& e0, const Eigen::Vector2f& e1);

Eigen::Vector4f point_point_dist_gradient(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1);

void point_line_distance_gradient_2D(float v01, float v02, float v11, float v12, float v21, float v22, float g[6]);

void point_line_distance_hessian_2D(float v01, float v02, float v11, float v12, float v21, float v22, float H[36]);

float minAlphaToPassThru(const Eigen::Vector2f& p, const Eigen::Vector2f& dir, const Eigen::Vector2f& a, const Eigen::Vector2f& b);

struct repulsivePair
{

    unsigned int pt_idx;
    unsigned int edge_idx;
    EPointEdgeDistanceType PE_type;

    repulsivePair(unsigned int i, unsigned j, EPointEdgeDistanceType petype);
};


#endif //IPC2DSIM_REPULSIONCALC_H
