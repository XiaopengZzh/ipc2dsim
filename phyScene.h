//
// Created by Xiaopeng Zhang on 3/23/24.
//

#ifndef IPC2DSIM_PHYSCENE_H
#define IPC2DSIM_PHYSCENE_H

#include "eigen-3.4.0/Eigen/Core"
#include "eigen-3.4.0/Eigen/Sparse"
#include "eigen-3.4.0/Eigen/SparseLU"
#include <iostream>
#include "neoHookeanCalc.h"
#include "repulsionCalc.h"

const float dmin = 0.001f;
const float stiffness = 1e5;
const float tolerance = 2e-2;
const Eigen::Vector2f gravity(0.0f, -9.81f);
const float yground = -10.0f;
const float dhat = 0.01f;
const float kappa = 1e5;
const float amplifier = 1e8;
const float E = 1e5f; // Young's module
//const float E = 1e4f;
const float nu = 0.4f; // Poisson ratio


class phyScene
{
public:
    phyScene() = default;

    std::vector<Eigen::Vector2f> vertices;
    std::vector<std::pair<unsigned int, unsigned int>> edges;
    std::vector<Eigen::Vector2f> velocities;
    std::vector<float> mass;
    std::vector<float> contactArea;
    std::vector<float> vol;
    std::vector<Eigen::Matrix2f> IB;
    std::vector<Eigen::Vector3i> eidx;
    std::vector<float> lams;
    std::vector<float> mus;

    // intermediate values
    std::vector<float> squaredRestLengths;
    std::vector<Eigen::Vector2f> verts_tilde;
    Eigen::VectorXf energyGradient;
    std::vector<Eigen::Triplet<float>> energyHessian;
    Eigen::VectorXf searchDir;
    std::vector<repulsivePair> repPairlst;

    void calcVertsTilde(float dt);

    float calcEnergy(float dt, float alpha = 0.0f);
    void calcEnergyGradient(float dt);
    void calcEnergyHessian(float dt);

    void calcSearchDir();

    void oneTimestepImpl(float dt);


    //=========================================
    float inertiaEnergyVal(float alpha);
    void calcInertiaEnergyGradient();
    void calcInertiaEnergyHessian();
    // ========================================

    float springEnergyVal(float alpha);
    void calcSpringEnergyGradient(float dt);
    void calcSpringEnergyHessian(float dt);
    // ========================================

    float gravEnergyVal(float alpha);
    void calcGravEnergyGradient(float dt);

    // ========================================

    float barrierEnergyVal(float alpha);
    void calcBarrierEnergyGradient(float dt);
    void calcBarrierEnergyHessian(float dt);

    float ccd();

    // ========================================

    Eigen::Matrix2f deformation_grad(unsigned int idx, float alpha = 0.0f);

    float lineSearchFilter();

    float neoHookeanEnergyVal(float alpha);
    void calcNeoHookeanEnergyGradient(float dt);
    void calcNeoHookeanEnergyHessian(float dt);

    // =====================================

    float repulsiveEnergyVal(float alpha);
    void calcRepulsiveEnergyGradient(float dt);
    void calcRepulsiveEnergyHessian(float dt);


    // =======

    bool b2TrianglesIntersect(unsigned int idx1, unsigned int idx2);
    bool bAnyPenetration();

    bool b2TrianglesIntersect_ccd(unsigned int idx1, unsigned int idx2, float alpha);
    bool bAnyPenetration_ccd(float alpha);

    float binarySearch_ccd(float low, float high, float precision);
};




#endif //IPC2DSIM_PHYSCENE_H
