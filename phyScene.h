//
// Created by Xiaopeng Zhang on 3/23/24.
//

#ifndef IPC2DSIM_PHYSCENE_H
#define IPC2DSIM_PHYSCENE_H

#include "eigen-3.4.0/Eigen/Core"

const float stiffness = 1e5;
//const float mass = 1.0f;



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
    //void oneTimestepImpl();

    // intermediate values
    std::vector<float> squaredRestLengths;
    std::vector<Eigen::Vector2f> verts_tilde;
    std::vector<float> energyGradient;
    sparseMat energyHessian;

    void calcVertsTilde(float dt);

    float calcEnergy(float dt, const Eigen::Vector2f& offset = Eigen::Vector2f(0.0f));
    void oneTimestepImpl(float dt);


    //=========================================
    float inertiaEnergyVal(const Eigen::Vector2f& offset = Eigen::Vector2f(0.0f));
    void calcInertiaEnergyGradient();
    void calcInertiaEnergyHessian();
    // ========================================

    float springEnergyVal(const Eigen::Vector2f& offset = Eigen::Vector2f(0.0f));
    void calcSpringEnergyGradient(float dt);
    void calcSpringEnergyHessian(float dt);

};




#endif //IPC2DSIM_PHYSCENE_H
