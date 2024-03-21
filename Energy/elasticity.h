//
// Created by Xiaopeng Zhang on 3/13/24.
//

#ifndef IPC2DSIM_ELASTICITY_H
#define IPC2DSIM_ELASTICITY_H

#include "../eigen-3.4.0/Eigen/Dense"


struct resPSVD
{
    Eigen::Vector2f sigma;
    Eigen::Matrix2f U;
    Eigen::Matrix2f V;
};

resPSVD polarSVD(const Eigen::Matrix2f& matF)
{
    resPSVD res;
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(matF, Eigen::ComputeFullU | Eigen::ComputeFullV);
    res.U = svd.matrixU();
    res.V = svd.matrixV();
    const auto& S = svd.singularValues();
    res.sigma[0] = S(0);
    res.sigma[1] = S(1);

    if(res.U.determinant() < 0)
    {
        res.U(0, 1) = -res.U(0, 1);
        res.U(1, 1) = -res.U(1, 1);
        res.sigma[1] = -res.sigma[1];
    }
    if(res.V.determinant() < 0)
    {
        res.V(0, 1) = -res.V(0, 1);
        res.V(1, 1) = -res.V(1, 1);
        res.sigma[1] = -res.sigma[1];
    }
    return res;
}





#endif //IPC2DSIM_ELASTICITY_H
