//
// Created by Xiaopeng Zhang on 3/27/24.
//

#include "repulsionCalc.h"

#include <cmath>

repulsivePair::repulsivePair(unsigned int i, unsigned int j, EPointEdgeDistanceType petype)
{
    pt_idx = i;
    edge_idx = j;
    PE_type = petype;
}

EPointEdgeDistanceType point_edge_dist_type(const Eigen::Vector2f& p, const Eigen::Vector2f& e0, const Eigen::Vector2f& e1)
{
    Eigen::Vector2f e = e1 - e0;
    const float e_length_sqr = e.squaredNorm();
    if(e_length_sqr == 0)
    {
        return EPointEdgeDistanceType::P_E;
    }

    const float ratio = e.dot(p - e0) / e_length_sqr;
    if(ratio < 0)
    {
        return EPointEdgeDistanceType::P_E0;
    }
    else if (ratio > 1)
    {
        return EPointEdgeDistanceType::P_E1;
    }
    else
    {
        return EPointEdgeDistanceType::P_E;
    }
}

Eigen::Vector4f point_point_dist_gradient(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1)
{
    Eigen::Vector4f res;
    Eigen::Vector2f temp = p0 - p1;
    res(0) = temp(0) * 2.0f;
    res(1) = temp(1) * 2.0f;
    res(2) = -res(0);
    res(3) = -res(1);
    return res;
}

void point_line_distance_gradient_2D(
        float v01,
        float v02,
        float v11,
        float v12,
        float v21,
        float v22,
        float g[6])
{
    float t13, t14, t23, t25, t24, t26, t27;

    t13 = -v21 + v11;
    t14 = -v22 + v12;
    t23 = 1.0f / (t13 * t13 + t14 * t14);
    t25 = ((v11 * v22 + -(v12 * v21)) + t14 * v01) + -(t13 * v02);
    t24 = t23 * t23;
    t26 = t25 * t25;
    t27 = (v11 * 2.0f + -(v21 * 2.0f)) * t24 * t26;
    t26 *= (v12 * 2.0f + -(v22 * 2.0f)) * t24;
    g[0] = t14 * t23 * t25 * 2.0f;
    g[1] = t13 * t23 * t25 * -2.0f;
    t24 = t23 * t25;
    g[2] = -t27 - t24 * (-v22 + v02) * 2.0f;
    g[3] = -t26 + t24 * (-v21 + v01) * 2.0f;
    g[4] = t27 + t24 * (v02 - v12) * 2.0f;
    g[5] = t26 - t24 * (v01 - v11) * 2.0f;
}

void point_line_distance_hessian_2D(
        float v01,
        float v02,
        float v11,
        float v12,
        float v21,
        float v22,
        float H[36])
{
    float t15, t16, t17, t18, t19, t20, t21, t22, t23, t24, t31, t34, t32,
            t33, t35, t60, t59, t62, t64, t65, t68, t71, t72, t75, t76, t77,
            t78, t79, t90, t92, t94, t96, t99, t93, t97, t98, t100, t102_tmp;

    t15 = -v11 + v01;
    t16 = -v12 + v02;
    t17 = -v21 + v01;
    t18 = -v22 + v02;
    t19 = -v21 + v11;
    t20 = -v22 + v12;
    t21 = v11 * 2.0f + -(v21 * 2.0f);
    t22 = v12 * 2.0f + -(v22 * 2.0f);
    t23 = t19 * t19;
    t24 = t20 * t20;
    t31 = 1.0f / (t23 + t24);
    t34 = ((v11 * v22 + -(v12 * v21)) + t20 * v01) + -(t19 * v02);
    t32 = t31 * t31;
    t33 = std::pow(t31, 3.0f);
    t35 = t34 * t34;
    t60 = t31 * t34 * 2.0f;
    t59 = -(t19 * t20 * t31 * 2.0f);
    t62 = t32 * t35 * 2.0f;
    t64 = t21 * t21 * t33 * t35 * 2.0f;
    t65 = t22 * t22 * t33 * t35 * 2.0f;
    t68 = t15 * t21 * t32 * t34 * 2.0f;
    t71 = t16 * t22 * t32 * t34 * 2.0f;
    t72 = t17 * t21 * t32 * t34 * 2.0f;
    t75 = t18 * t22 * t32 * t34 * 2.0f;
    t76 = t19 * t21 * t32 * t34 * 2.0f;
    t77 = t20 * t21 * t32 * t34 * 2.0f;
    t78 = t19 * t22 * t32 * t34 * 2.0f;
    t79 = t20 * t22 * t32 * t34 * 2.0f;
    t90 = t21 * t22 * t33 * t35 * 2.0f;
    t92 = t16 * t20 * t31 * 2.0f + t77;
    t94 = -(t17 * t19 * t31 * 2.0f) + t78;
    t96 = (t18 * t19 * t31 * 2.0f + -t60) + t76;
    t99 = (-(t15 * t20 * t31 * 2.0f) + -t60) + t79;
    t93 = t15 * t19 * t31 * 2.0f + -t78;
    t35 = -(t18 * t20 * t31 * 2.0f) + -t77;
    t97 = (t17 * t20 * t31 * 2.0f + t60) + -t79;
    t98 = (-(t16 * t19 * t31 * 2.0f) + t60) + -t76;
    t100 = ((-(t15 * t16 * t31 * 2.0f) + t71) + -t68) + t90;
    t19 = ((-(t17 * t18 * t31 * 2.0f) + t75) + -t72) + t90;
    t102_tmp = t17 * t22 * t32 * t34;
    t76 = t15 * t22 * t32 * t34;
    t22 = (((-(t15 * t17 * t31 * 2.0f) + t62) + -t65) + t76 * 2.0f)
          + t102_tmp * 2.0f;
    t33 = t18 * t21 * t32 * t34;
    t20 = t16 * t21 * t32 * t34;
    t79 = (((-(t16 * t18 * t31 * 2.0f) + t62) + -t64) + -(t20 * 2.0f))
          + -(t33 * 2.0f);
    t77 = (((t15 * t18 * t31 * 2.0f + t60) + t68) + -t75) + -t90;
    t78 = (((t16 * t17 * t31 * 2.0f + -t60) + t72) + -t71) + -t90;
    H[0] = t24 * t31 * 2.0f;
    H[1] = t59;
    H[2] = t35;
    H[3] = t97;
    H[4] = t92;
    H[5] = t99;
    H[6] = t59;
    H[7] = t23 * t31 * 2.0f;
    H[8] = t96;
    H[9] = t94;
    H[10] = t98;
    H[11] = t93;
    H[12] = t35;
    H[13] = t96;
    t35 = -t62 + t64;
    H[14] = (t35 + t18 * t18 * t31 * 2.0f) + t33 * 4.0f;
    H[15] = t19;
    H[16] = t79;
    H[17] = t77;
    H[18] = t97;
    H[19] = t94;
    H[20] = t19;
    t33 = -t62 + t65;
    H[21] = (t33 + t17 * t17 * t31 * 2.0f) - t102_tmp * 4.0f;
    H[22] = t78;
    H[23] = t22;
    H[24] = t92;
    H[25] = t98;
    H[26] = t79;
    H[27] = t78;
    H[28] = (t35 + t16 * t16 * t31 * 2.0f) + t20 * 4.0f;
    H[29] = t100;
    H[30] = t99;
    H[31] = t93;
    H[32] = t77;
    H[33] = t22;
    H[34] = t100;
    H[35] = (t33 + t15 * t15 * t31 * 2.0f) - t76 * 4.0f;
}


float minAlphaToPassThru(const Eigen::Vector2f& p, const Eigen::Vector2f& dir, const Eigen::Vector2f& a, const Eigen::Vector2f& b)
{
    if(a == b)
    {
        return 1.0f;
    }
    if(dir == Eigen::Vector2f::Zero())
    {
        return 1.0f;
    }
    Eigen::Vector2f N;
    N(0) = (b - a).y();
    N(1) = (a - b).x();
    float t1 = (a - p).dot(N);
    float t2 = dir.dot(N);
    if(abs(t2) < 1e-2 || abs(t1) < 1e-2)
    {
        return 1.0f;
    }
    float t = (a - p).dot(N) / dir.dot(N);
    if(t < 0 || t > 1.0f)
    {
        return 1.0f;
    }
    float ttt = 1.0f;
    return t * 0.9f;
}

float repulsion(float d, float dhat)
{
    if(d <= 0.0f)
    {
        return std::numeric_limits<float>::infinity();
    }
    if(d >= dhat)
    {
        return 0.0f;
    }
    return -(d - dhat) * (d - dhat) * std::log(d / dhat);
}

float repulsionFirstDerivative(float d, float dhat)
{
    if (d <= 0.0f || d >= dhat) {
        return 0.0f;
    }

    return (dhat - d) * (2 * std::log(d / dhat) - dhat / d + 1);
}


float repulsionSecondDerivative(float d, float dhat)
{
    if (d <= 0.0f || d >= dhat) {
        return 0.0f;
    }
    const float dhat_d = dhat / d;
    return (dhat_d + 2) * dhat_d - 2 * std::log(d / dhat) - 3;
}

// in-place operation
void spdProjection6x6(float b[36])
{
    Eigen::Map<Eigen::Matrix<float, 6, 6, Eigen::RowMajor>> mat(b);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6, Eigen::RowMajor>> solver(mat);
    Eigen::VectorXf lam = solver.eigenvalues();
    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> V = solver.eigenvectors();

    for(int i = 0; i < lam.size(); i++)
    {
        lam(i) = std::max(0.0f, lam(i));
    }

    Eigen::Matrix<float, 6, 6, Eigen::RowMajor> diagLam = lam.asDiagonal();

    mat = V * diagLam * V.transpose();
}

