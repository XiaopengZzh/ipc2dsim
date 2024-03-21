//
// Created by Xiaopeng Zhang on 3/14/24.
//

#ifndef IPC2DSIM_SPARSEMATRIX_H
#define IPC2DSIM_SPARSEMATRIX_H
#include <vector>
#include "../eigen-3.4.0/Eigen/Dense"

struct sparseMat
{
    std::vector<int> loc_i;
    std::vector<int> loc_j;
    std::vector<float> vals;
};

Eigen::Matrix4f spdProjection(Eigen::Matrix4f& hess)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4f> solver(hess);
    Eigen::Vector4f lam = solver.eigenvalues();
    Eigen::Matrix4f V = solver.eigenvectors();

    for(int i = 0; i < lam.size(); i++)
    {
        lam(i) = std::max(0.0f, lam(i));
    }

    Eigen::Matrix4f diagLam = lam.asDiagonal();
    Eigen::Matrix4f spd = V * diagLam * V.transpose();
    return spd;
}




#endif //IPC2DSIM_SPARSEMATRIX_H
