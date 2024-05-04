//
// Created by Xiaopeng Zhang on 3/26/24.
//

#ifndef IPC2DSIM_NEOHOOKEANCALC_H
#define IPC2DSIM_NEOHOOKEANCALC_H

#include "Eigen/Core"
#include <vector>
#include "Eigen/Dense"
#include <cmath>

Eigen::Matrix4f spdProjection(const Eigen::Matrix4f& hess);
Eigen::Matrix2f spdProjection2d(const Eigen::Matrix2f& hess);

float Psi(const Eigen::Matrix2f& F, float mu, float lam);

std::tuple<Eigen::Matrix2f, Eigen::Vector2f, Eigen::Matrix2f> polarSVD(const Eigen::Matrix2f& F);

Eigen::Vector2f dPsi_dSigma(const Eigen::Vector2f& s, float mu, float lam);

Eigen::Matrix2f d2Psi_dSigma2(const Eigen::Vector2f& s, float mu, float lam);

float B_left_coef(const Eigen::Vector2f& s, float mu, float lam);

Eigen::Matrix2f dPsi_dF(const Eigen::Matrix2f& F, float mu, float lam);

Eigen::Matrix4f d2Psi_dF2(const Eigen::Matrix2f& F, float mu, float lam);

std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f> dPsi_dx(const Eigen::Matrix2f& P, const Eigen::Matrix2f& IB);

Eigen::MatrixXf d2Psi_dx2(const Eigen::Matrix4f& dPdF, const Eigen::Matrix2f& IB);

float sprRoot(float a, float b, float c, float tol = 1e-6f);

#endif //IPC2DSIM_NEOHOOKEANCALC_H
