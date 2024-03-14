//
// Created by Xiaopeng Zhang on 3/13/24.
//

#ifndef IPC2DSIM_UTILS_H
#define IPC2DSIM_UTILS_H

#include <iostream>
#include "eigen-3.4.0/Eigen/Dense"
#include <glad.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"


struct resSVD
{
    glm::vec2 sigma;
    glm::mat2x2 U;
    glm::mat2x2 V;
};

resSVD SVD(glm::mat2x2 matrix)
{
    resSVD res;
    Eigen::Matrix2d mat;
    mat(0, 0) = matrix[0][0];
    mat(0, 1) = matrix[0][1];
    mat(1, 0) = matrix[1][0];
    mat(1, 1) = matrix[1][1];

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& U = svd.matrixU();
    const auto& S = svd.singularValues();
    const auto& V = svd.matrixV();

    //std::cout << U << std::endl;
    res.sigma[0] = S(0);
    res.sigma[1] = S(1);
    res.U[0][0] = U(0, 0);
    res.U[0][1] = U(0, 1);
    res.U[1][0] = U(1, 0);
    res.U[1][1] = U(1, 1);
    res.V[0][0] = V(0, 0);
    res.V[1][0] = V(1, 0);
    res.V[0][1] = V(0, 1);
    res.V[1][1] = V(1, 1);

    return res;
}







#endif //IPC2DSIM_UTILS_H
