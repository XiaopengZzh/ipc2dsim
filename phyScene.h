//
// Created by Xiaopeng Zhang on 3/23/24.
//

#ifndef IPC2DSIM_PHYSCENE_H
#define IPC2DSIM_PHYSCENE_H

#include "eigen-3.4.0/Eigen/Core"
#include "eigen-3.4.0/Eigen/Sparse"
#include "eigen-3.4.0/Eigen/SparseLU"
#include <iostream>


const float stiffness = 1e5;
//const float mass = 1.0f;
const float tolerance = 1e-2;


struct sparseMat
{
    std::vector<unsigned int> loc_i;
    std::vector<unsigned int> loc_j;
    std::vector<float> vals;
};

class phyScene
{
public:
    phyScene() = default;

    std::vector<Eigen::Vector2f> vertices;
    std::vector<std::pair<unsigned int, unsigned int>> edges;
    std::vector<Eigen::Vector2f> velocities;

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

};




#endif //IPC2DSIM_PHYSCENE_H