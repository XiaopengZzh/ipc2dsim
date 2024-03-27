//
// Created by Xiaopeng Zhang on 3/23/24.
//

#include "phyScene.h"
#include "eigen-3.4.0/Eigen/Dense"
#include <cmath>
#include "eigen-3.4.0/unsupported/Eigen/src/IterativeSolvers/Scaling.h"
//#include "eigen-3.4.0/Eigen/src/OrderingMethods/Ordering.h"

float infnorm(const Eigen::VectorXf& vec)
{
    unsigned int sz = vec.size();
    float res = vec[0];
    for(unsigned int i = 0; i < sz; i = i + 2)
    {
        if(res < abs(vec[i]) + abs(vec[i + 1]) )
        {
            res = abs(vec[i]) + abs(vec[i + 1]);
        }
    }
    return res;
}


void phyScene::calcVertsTilde(float dt)
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        verts_tilde[i] = vertices[i] + dt * velocities[i];
    }
}

float phyScene::calcEnergy(float dt, const float alpha)
{
    return inertiaEnergyVal(alpha) + dt * dt * (gravEnergyVal(alpha) + barrierEnergyVal(alpha) +
            neoHookeanEnergyVal(alpha));
}

void phyScene::calcEnergyGradient(float dt)
{
    calcInertiaEnergyGradient();
    //calcSpringEnergyGradient(dt);
    calcGravEnergyGradient(dt);
    calcBarrierEnergyGradient(dt);
    calcNeoHookeanEnergyGradient(dt);
}

void phyScene::calcEnergyHessian(float dt)
{
    calcInertiaEnergyHessian();
    //calcSpringEnergyHessian(dt);
    calcNeoHookeanEnergyHessian(dt);
    calcBarrierEnergyHessian(dt);
}

void phyScene::calcSearchDir()
{
    unsigned int dim = 2 * vertices.size();

    //==========
    Eigen::VectorXd minusd_grad(energyGradient.size());

    for(unsigned int i = 0; i < energyGradient.size(); i++)
    {
        minusd_grad(i) = static_cast<double>(-energyGradient(i));
    }
    //======
    std::vector<Eigen::Triplet<double>> db_hess;
    db_hess.reserve(energyHessian.size());
    for(auto i : energyHessian)
    {
        db_hess.emplace_back(i.row(), i.col(), static_cast<double>(i.value()));
    }
    // ========

    Eigen::SparseMatrix<double> hessian(dim, dim);
    hessian.setFromTriplets(db_hess.begin(), db_hess.end());

    Eigen::IterScaling<Eigen::SparseMatrix<double>> scal;
    scal.computeRef(hessian);
    //Eigen::VectorXd b = minusd_grad;
    minusd_grad = scal.LeftScaling().cwiseProduct(minusd_grad);

    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    //solver
    solver.analyzePattern(hessian);
    solver.factorize(hessian);

    Eigen::VectorXd direction = solver.solve(minusd_grad);
    direction = scal.RightScaling().cwiseProduct(direction);

    for(unsigned int i = 0; i < direction.size(); i++)
    {
        searchDir(i) = static_cast<float>(direction(i));
    }

    //searchDir = solver.solve(-energyGradient);
}

void phyScene::oneTimestepImpl(float dt)
{
    energyGradient.setZero();
    energyHessian.clear();

    std::vector<Eigen::Vector2f> prevVertices = vertices;

    calcVertsTilde(dt);

    unsigned int iter = 0;

    float E_last = calcEnergy(dt);
    calcEnergyGradient(dt);
    calcEnergyHessian(dt);
    calcSearchDir();

    // debug
    /*
    std::vector<float> temp1;
    for(float i : energyGradient)
    {
        temp1.push_back(i);
    }
    std::vector<float> temp2;
    for(float i : searchDir)
    {
        temp2.push_back(i);
    }
    */

    while(infnorm(searchDir) > tolerance * dt)
    {
        printf("   Iteration %d : ", iter);
        printf("residual = %f, ", infnorm(searchDir) / dt);

        float alpha = 1.0f;

        //float a = calcEnergy(dt, alpha);
        alpha = std::min(ccd(), lineSearchFilter());

        while(calcEnergy(dt, alpha) > E_last)
        {
            alpha /= 2.0f;
        }
        printf("step size = %f\n", alpha);

        for(unsigned int i = 0; i < vertices.size(); i++)
        {
            vertices[i] += alpha * Eigen::Vector2f(searchDir[2 * i], searchDir[2 * i + 1]);
        }

        E_last = calcEnergy(dt);
        energyGradient.setZero();
        energyHessian.clear();
        calcEnergyGradient(dt);
        calcEnergyHessian(dt);
        calcSearchDir();

        //debug
        /*
        std::vector<float> temp3;
        for(float i : energyGradient)
        {
            temp3.push_back(i);
        }
        std::vector<float> temp4;
        for(float i : searchDir)
        {
            temp4.push_back(i);
        }
        */

        iter++;
    }

    // update velocities
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        velocities[i] = (vertices[i] - prevVertices[i]) / dt;
    }
}


// todo : integrating mass, spring stiffness
float phyScene::inertiaEnergyVal(const float alpha)
{
    float sum = 0.0f;
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector2f offset(searchDir[2 * i], searchDir[2 * i + 1]);
        Eigen::Vector2f diff = vertices[i] - verts_tilde[i] + alpha * offset;
        sum += 0.5f * diff.dot(diff) * mass[i];
    }
    return sum;
}

void phyScene::calcInertiaEnergyGradient()
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector2f diff = vertices[i] - verts_tilde[i];
        energyGradient[2 * i] += diff.x() * mass[i];
        energyGradient[2 * i + 1] += diff.y() * mass[i];
    }
}

void phyScene::calcInertiaEnergyHessian()
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        energyHessian.emplace_back(2 * i, 2 * i, mass[i]);
        energyHessian.emplace_back(2 * i + 1, 2 * i + 1, mass[i]);
    }
}

//=====================================

float phyScene::springEnergyVal(const float alpha)
{
    float sum = 0.0f;
    for(unsigned int i = 0; i < edges.size(); i++)
    {
        unsigned int first = edges[i].first;
        unsigned int second = edges[i].second;
        Eigen::Vector2f offset(searchDir[2 * first] - searchDir[2 * second],
                               searchDir[2 * first + 1] - searchDir[2 * second + 1]);
        Eigen::Vector2f diff = vertices[first] - vertices[second] + alpha * offset;
        float temp = diff.dot(diff) / squaredRestLengths[i] - 1;
        sum += 0.5f * squaredRestLengths[i] * temp * temp * stiffness;
    }
    return sum;
}

void phyScene::calcSpringEnergyGradient(float dt)
{
    for(unsigned int i = 0; i < edges.size(); i++)
    {
        Eigen::Vector2f diff = vertices[edges[i].first] - vertices[edges[i].second];
        float factor = 2 * stiffness * (diff.dot(diff) / squaredRestLengths[i] - 1);
        diff = dt * dt * factor * diff;
        energyGradient[2 * edges[i].first] += diff.x();
        energyGradient[2 * edges[i].first + 1] += diff.y();
        energyGradient[2 * edges[i].second] -= diff.x();
        energyGradient[2 * edges[i].second + 1] -= diff.y();
    }
}

void phyScene::calcSpringEnergyHessian(float dt)
{
    for(unsigned int i = 0; i < edges.size(); i++)
    {
        unsigned int first = edges[i].first;
        unsigned int second = edges[i].second;
        Eigen::Vector2f diff = vertices[first] - vertices[second];
        Eigen::Matrix2f H_diff = 2 * stiffness / squaredRestLengths[i] * (
                2 * diff * diff.transpose() + (diff.dot(diff) - squaredRestLengths[i]) * Eigen::Matrix2f::Identity()
                );
        Eigen::Matrix4f integration;
        integration.topLeftCorner(2, 2) = H_diff;
        integration.topRightCorner(2, 2) = -H_diff;
        integration.bottomLeftCorner(2, 2) = - H_diff;
        integration.bottomRightCorner(2, 2) = H_diff;
        Eigen::Matrix4f H_local = dt * dt * spdProjection(integration);

        for(unsigned int r = 0; r < 2; r++)
        {
            for(unsigned int c = 0; c < 2; c++)
            {
                energyHessian.emplace_back(first * 2 + r, first * 2 + c, H_local(r, c));
                energyHessian.emplace_back(first * 2 + r, second * 2 + c, H_local(r, c + 2));
                energyHessian.emplace_back(second * 2 + r, first * 2 + c, H_local(r + 2, c));
                energyHessian.emplace_back(second * 2 + r, second * 2 + c, H_local(r + 2, c + 2));
            }
        }
    }
}

float phyScene::gravEnergyVal(float alpha)
{
    float sum = 0.0f;
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector2f offset(searchDir[2 * i], searchDir[2 * i + 1]);
        Eigen::Vector2f x = vertices[i] + alpha * offset;
        sum += -mass[i] * x.dot(gravity);
    }
    return sum;
}

void phyScene::calcGravEnergyGradient(float dt)
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        energyGradient[2 * i] -= mass[i] * gravity[0] * dt * dt;
        energyGradient[2 * i + 1] -= mass[i] * gravity[1] * dt * dt;
    }
}

float phyScene::barrierEnergyVal(float alpha)
{
    float sum = 0.0f;
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        float d = vertices[i].y() - yground + alpha * searchDir[2 * i + 1];
        if(d < dhat)
        {
            float s = d / dhat;
            sum += dhat * kappa / 2 * (s - 1) * log(s) * contactArea[i];
        }
    }
    return sum;
}

void phyScene::calcBarrierEnergyGradient(float dt)
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        float d = vertices[i].y() - yground;
        if(d < dhat)
        {
            float s = d / dhat;
            energyGradient[2 * i + 1] += dt * dt * contactArea[i] * dhat * (kappa / 2 * (log(s) / dhat + (s - 1) / d));
        }
    }
}

void phyScene::calcBarrierEnergyHessian(float dt)
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        float d = vertices[i].y() - yground;
        float hess = 0.0f;
        if(d < dhat)
        {
            hess = contactArea[i] * dhat * kappa / (2 * d * d * dhat) * (d + dhat) * dt * dt;
            energyHessian.emplace_back(2 * i + 1, 2 * i + 1, hess);
        }
    }
}

float phyScene::ccd()
{
    float alpha = 1.0f;
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        if(searchDir[2 * i + 1] < 0)
        {
            alpha = std::min(alpha, 0.9f * (yground - vertices[i].y()) / searchDir[2 * i + 1]);
        }
    }
    return alpha;
}

Eigen::Matrix2f phyScene::deformation_grad(unsigned int idx, float alpha)
{
    Eigen::Matrix2f F;
    unsigned int e0, e1, e2;
    e0 = eidx[idx](0);
    e1 = eidx[idx](1);
    e2 = eidx[idx](2);
    Eigen::Vector2f p0(searchDir[2 * e0], searchDir[2 * e0 + 1]);
    Eigen::Vector2f p1(searchDir[2 * e1], searchDir[2 * e1 + 1]);
    Eigen::Vector2f p2(searchDir[2 * e2], searchDir[2 * e2 + 1]);
    Eigen::Vector2f x0 = vertices[e0] + alpha * p0;
    Eigen::Vector2f x1 = vertices[e1] + alpha * p1;
    Eigen::Vector2f x2 = vertices[e2] + alpha * p2;
    F.col(0) = x1 - x0;
    F.col(1) = x2 - x0;
    return F * IB[idx];
}

float phyScene::lineSearchFilter()
{
    float alpha = 1.0f;

    for(auto & i : eidx)
    {
        unsigned int e0 = i(0);
        unsigned int e1 = i(1);
        unsigned int e2 = i(2);
        Eigen::Vector2f x21 = vertices[e1] - vertices[e0];
        Eigen::Vector2f x31 = vertices[e2] - vertices[e0];
        Eigen::Vector2f p21(searchDir[2 * e1] - searchDir[2 * e0], searchDir[2 * e1 + 1] - searchDir[2 * e0 + 1]);
        Eigen::Vector2f p31(searchDir[2 * e2] - searchDir[2 * e0], searchDir[2 * e2 + 1] - searchDir[2 * e0 + 1]);

        Eigen::Matrix2f P, X, temp1, temp2;
        P.col(0) = p21;
        P.col(1) = p31;
        X.col(0) = x21;
        X.col(1) = x31;
        temp1.col(0) = x21;
        temp1.col(1) = p31;
        temp2.col(0) = p21;
        temp2.col(1) = x31;

        float detX = X.determinant();
        float a = P.determinant() / detX;
        float b = (temp1.determinant() + temp2.determinant()) / detX;
        float c = 0.9;

        float solvedAlpha = sprRoot(a, b, c);
        if(solvedAlpha > 0)
        {
            alpha = std::min(alpha, solvedAlpha);
        }

    }

    return alpha;
}

float phyScene::neoHookeanEnergyVal(float alpha)
{
    float sum = 0.0f;

    for(unsigned int i = 0; i < eidx.size(); i++)
    {
        Eigen::Matrix2f F = deformation_grad(i, alpha);
        sum += vol[i] * Psi(F, mus[i], lams[i]);
    }

    return sum;
}

void phyScene::calcNeoHookeanEnergyGradient(float dt)
{
    for(unsigned int i = 0; i < eidx.size(); i++)
    {
        Eigen::Matrix2f F = deformation_grad(i);
        Eigen::Matrix2f P = vol[i] * dPsi_dF(F, mus[i], lams[i]);
        Eigen::Vector2f local1, local2, local0;
        std::tie(local0, local1, local2) = dPsi_dx(P, IB[i]);
        unsigned int e0 = eidx[i](0);
        unsigned int e1 = eidx[i](1);
        unsigned int e2 = eidx[i](2);
        energyGradient(2 * e0) += local0(0) * dt * dt;
        energyGradient(2 * e0 + 1) += local0(1) * dt * dt;
        energyGradient(2 * e1) += local1(0) * dt * dt;
        energyGradient(2 * e1 + 1) += local1(1) * dt * dt;
        energyGradient(2 * e2) += local2(0) * dt * dt;
        energyGradient(2 * e2 + 1) += local2(1) * dt * dt;
    }
}

void phyScene::calcNeoHookeanEnergyHessian(float dt)
{
    for(unsigned int i = 0; i < eidx.size(); i++)
    {
        Eigen::Matrix2f F = deformation_grad(i);
        Eigen::Matrix4f dPdF = vol[i] * d2Psi_dF2(F, mus[i], lams[i]);
        Eigen::MatrixXf local_hess = d2Psi_dx2(dPdF, IB[i]);
        for(unsigned int xI = 0; xI < 3; xI++)
        {
            for(unsigned int xJ = 0; xJ < 3; xJ++)
            {
                for(unsigned int dI = 0; dI < 2; dI++)
                {
                    for(unsigned int dJ = 0; dJ < 2; dJ++)
                    {
                        unsigned int idxi = eidx[i](xI) * 2 + dI;
                        unsigned int idxj = eidx[i](xJ) * 2 + dJ;
                        float val = local_hess(xI * 2 + dI, xJ * 2 + dJ) * dt * dt;
                        energyHessian.emplace_back(idxi, idxj, val);
                    }
                }
            }
        }
    }
}



