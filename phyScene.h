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


const float stiffness = 1e5;
const float tolerance = 1e-2;
const Eigen::Vector2f gravity(0.0f, -0.8f);
const float yground = -3.0f;
const float dhat = 0.1f;
const float kappa = 1e5;
const float E = 1e5f; // Young's module
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

};




#endif //IPC2DSIM_PHYSCENE_H
