//
// Created by Xiaopeng Zhang on 3/27/24.
//

#ifndef IPC2DSIM_REPULSIONCALC_H
#define IPC2DSIM_REPULSIONCALC_H

#include "Eigen/Core"
#include "Eigen/Dense"

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

float repulsion(float d, float dhat);
float repulsionFirstDerivative(float d, float dhat);
float repulsionSecondDerivative(float d, float dhat);

void spdProjection6x6(float b[36]);

float crossProduct(const Eigen::Vector2f& v, const Eigen::Vector2f& a, const Eigen::Vector2f& b);

#endif //IPC2DSIM_REPULSIONCALC_H
