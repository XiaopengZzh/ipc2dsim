//
// Created by Xiaopeng Zhang on 3/26/24.
//

#include "neoHookeanCalc.h"


Eigen::Matrix4f spdProjection(const Eigen::Matrix4f& hess)
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

Eigen::Matrix2f spdProjection2d(const Eigen::Matrix2f& F)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(F);
    Eigen::Vector2f lam = solver.eigenvalues();
    Eigen::Matrix2f V = solver.eigenvectors();

    for(int i = 0; i < lam.size(); i++)
    {
        lam(i) = std::max(0.0f, lam(i));
    }

    Eigen::Matrix2f diagLam = lam.asDiagonal();
    Eigen::Matrix2f spd = V * diagLam * V.transpose();
    return spd;
}



std::tuple<Eigen::Matrix2f, Eigen::Vector2f, Eigen::Matrix2f> polarSVD(const Eigen::Matrix2f& F)
{
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2f U = svd.matrixU();
    Eigen::Vector2f s = svd.singularValues();
    Eigen::Matrix2f VT = svd.matrixV().transpose();
    if(U.determinant() < 0)
    {
        U.col(1) = -U.col(1);
        s(1) = -s(1);
    }
    if(VT.determinant() < 0)
    {
        VT.row(1) = -VT.row(1);
        s(1) = -s(1);
    }
    return {U, s, VT};
}

Eigen::Vector2f dPsi_dSigma(const Eigen::Vector2f& s, float mu, float lam)
{
    float prod = log(s(0) * s(1));
    float inv0 = 1.0f / s(0);
    float inv1 = 1.0f / s(1);
    float res0 = mu * (s(0) - inv0) + lam * inv0 * prod;
    float res1 = mu * (s(1) - inv1) + lam * inv1 * prod;
    return Eigen::Vector2f{res0, res1};
}

Eigen::Matrix2f d2Psi_dSigma2(const Eigen::Vector2f& s, float mu, float lam)
{
    float prod = log(s(0) * s(1));
    float inv2_0 = 1 / (s(0) * s(0));
    float inv2_1 = 1 / (s(1) * s(1));
    float res00 = mu * (1 + inv2_0) - lam * inv2_0 * (prod - 1);
    float res11 = mu * (1 + inv2_1) - lam * inv2_1 * (prod - 1);
    float res01 = lam / (s(0) * s(1));
    Eigen::Matrix2f res;
    res << res00, res01, res01, res11;
    return res;
}

float B_left_coef(const Eigen::Vector2f& s, float mu, float lam)
{
    float prod = s(0) * s(1);
    return (mu + (mu - lam * log(prod)) / prod) / 2;
}

Eigen::Matrix4f d2Psi_dF2(const Eigen::Matrix2f& F, float mu, float lam)
{
    Eigen::Matrix2f U, VT;
    Eigen::Vector2f sigma;
    std::tie(U, sigma, VT) = polarSVD(F);

    Eigen::Matrix2f Psi_sigma_sigma = spdProjection2d(d2Psi_dSigma2(sigma, mu, lam));
    float B_left = B_left_coef(sigma, mu, lam);

    Eigen::Vector2f Psi_sigma = dPsi_dSigma(sigma, mu, lam);
    float B_right = (Psi_sigma(0) + Psi_sigma(1)) / (2 * std::max(sigma(0) + sigma(1), 1e-6f));
    Eigen::Matrix2f temp;
    temp << B_left + B_right, B_left - B_right, B_left - B_right, B_left + B_right;
    Eigen::Matrix2f B = spdProjection2d(temp);

    Eigen::Matrix4f M = Eigen::Matrix4f::Zero();
    M(0, 0) = Psi_sigma_sigma(0, 0);
    M(0, 3) = Psi_sigma_sigma(0, 1);
    M(1, 1) = B(0, 0);
    M(1, 2) = B(0, 1);
    M(2, 1) = B(1, 0);
    M(2, 2) = B(1, 1);
    M(3, 0) = Psi_sigma_sigma(1, 0);
    M(3, 3) = Psi_sigma_sigma(1, 1);

    Eigen::Matrix4f res = Eigen::Matrix4f::Zero();
    for(unsigned int j = 0; j < 2; j++)
    {
        for(unsigned int i = 0; i < 2; i++)
        {
            unsigned int ij = j * 2 + i;
            for(unsigned int s = 0; s < 2; s++)
            {
                for(unsigned int r = 0; r < 2; r++)
                {
                    unsigned int rs = s * 2 + r;
                    res(ij, rs) = M(0, 0) * U(i, 0) * VT(0, j) * U(r, 0) * VT(0, s) +
                            M(0, 3) * U(i, 0) * VT(0, j) * U(r, 1) * VT(1, s) +
                            M(1, 1) * U(i, 0) * VT(1, j) * U(r, 0) * VT(1, s) +
                            M(1, 2) * U(i, 0) * VT(1, j) * U(r, 1) * VT(0, s) +
                            M(2, 1) * U(i, 1) * VT(0, j) * U(r, 0) * VT(1, s) +
                            M(2, 2) * U(i, 1) * VT(0, j) * U(r, 1) * VT(0, s) +
                            M(3, 0) * U(i, 1) * VT(1, j) * U(r, 0) * VT(0, s) +
                            M(3, 3) * U(i, 1) * VT(1, j) * U(r, 1) * VT(1, s);
                }
            }
        }
    }
    return res;
}

std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f> dPsi_dx(const Eigen::Matrix2f& P, const Eigen::Matrix2f& IB)
{
    float d2 = P(0, 0) * IB(0, 0) + P(0, 1) * IB(0, 1);
    float d3 = P(1, 0) * IB(0, 0) + P(1, 1) * IB(0, 1);
    float d4 = P(0, 0) * IB(1, 0) + P(0, 1) * IB(1, 1);
    float d5 = P(1, 0) + IB(1, 0) + P(1, 1) * IB(1, 1);
    Eigen::Vector2f res0, res1, res2;
    res0(0) = -d2 - d4;
    res0(1) = -d3 - d5;
    res1(0) = d2;
    res1(1) = d3;
    res2(0) = d4;
    res2(1) = d5;
    return {res0, res1, res2};
}

Eigen::MatrixXf d2Psi_dx2(const Eigen::Matrix4f& dPdF, const Eigen::Matrix2f& IB)
{
    Eigen::MatrixXf itm = Eigen::MatrixXf::Zero(6, 4);
    for(unsigned int colI = 0; colI < 4; colI++)
    {
        float _000 = dPdF(0, colI) * IB(0, 0);
        float _010 = dPdF(0, colI) * IB(1, 0);
        float _101 = dPdF(2, colI) * IB(0, 1);
        float _111 = dPdF(2, colI) * IB(1, 1);
        float _200 = dPdF(1, colI) * IB(0, 0);
        float _210 = dPdF(1, colI) * IB(1, 0);
        float _301 = dPdF(3, colI) * IB(0, 1);
        float _311 = dPdF(3, colI) * IB(1, 1);
        itm(2, colI) = _000 + _101;
        itm(3, colI) = _200 + _301;
        itm(4, colI) = _010 + _111;
        itm(5, colI) = _210 + _311;
        itm(0, colI) = -itm(2, colI) - itm(4, colI);
        itm(1, colI) = -itm(3, colI) - itm(5, colI);
    }
    Eigen::MatrixXf res = Eigen::MatrixXf::Zero(6, 6);
    for(unsigned int colI = 0; colI < 6; colI++)
    {
        float _000 = itm(colI, 0) * IB(0, 0);
        float _010 = itm(colI, 0) * IB(1, 0);
        float _101 = itm(colI, 2) * IB(0, 1);
        float _111 = itm(colI, 2) * IB(1, 1);
        float _200 = itm(colI, 1) * IB(0, 0);
        float _210 = itm(colI, 1) * IB(1, 0);
        float _301 = itm(colI, 3) * IB(0, 1);
        float _311 = itm(colI, 3) * IB(1, 1);
        res(2, colI) = _000 + _101;
        res(3, colI) = _200 + _301;
        res(4, colI) = _010 + _111;
        res(5, colI) = _210 + _311;
        res(0, colI) = -_000 - _101 - _010 - _111;
        res(1, colI) = -_200 - _301 - _210 - _311;
    }
    return res;
}


