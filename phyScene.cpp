//
// Created by Xiaopeng Zhang on 3/23/24.
//

#include "phyScene.h"
#include "Eigen/Dense"
#include <cmath>
//#include "eigen-3.4.0/unsupported/Eigen/src/IterativeSolvers/Scaling.h"
//#include "eigen-3.4.0/Eigen/src/OrderingMethods/Ordering.h"

extern bool bUnitTest;
extern bool bPenetration;
extern unsigned int checkedIdx;

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
            neoHookeanEnergyVal(alpha) + repulsiveEnergyVal(alpha));
}

void phyScene::calcEnergyGradient(float dt)
{
    calcInertiaEnergyGradient();
    //calcSpringEnergyGradient(dt);
    calcGravEnergyGradient(dt);
    calcBarrierEnergyGradient(dt);
    calcNeoHookeanEnergyGradient(dt);
    calcRepulsiveEnergyGradient(dt);
}

void phyScene::calcEnergyHessian(float dt)
{
    calcInertiaEnergyHessian();
    //calcSpringEnergyHessian(dt);
    calcNeoHookeanEnergyHessian(dt);
    calcBarrierEnergyHessian(dt);
    calcRepulsiveEnergyHessian(dt);
}

void phyScene::calcSearchDir()
{
    unsigned int dim = 2 * vertices.size();
    Eigen::SparseMatrix<float> hessian(dim, dim);
    hessian.setFromTriplets(energyHessian.begin(), energyHessian.end());
    Eigen::ConjugateGradient<Eigen::SparseMatrix<float>, Eigen::Lower | Eigen::Upper> cg;
    cg.compute(hessian);
    searchDir = cg.solve(-energyGradient);
    //Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
    //solver
    //solver.analyzePattern(hessian);
    //solver.factorize(hessian);
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


        iter++;
    }

    // update velocities
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        velocities[i] = (vertices[i] - prevVertices[i]) / dt;
    }

    // for unit tests
    if(bUnitTest)
    {
        if(bAnyPenetration())
        {
            bPenetration = true;
        }
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
            sum += dhat * kappa / 2 * (s - 1) * std::log(s) * contactArea[i];
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
            energyGradient[2 * i + 1] += dt * dt * contactArea[i] * dhat * (kappa / 2 * (std::log(s) / dhat + (s - 1) / d));
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

    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        for(auto edge : edges)
        {
            if(edge.first == i || edge.second == i)
            {
                continue;
            }
            Eigen::Vector2f dir;
            dir(0) = searchDir(2 * i);
            dir(1) = searchDir(2 * i + 1);
            float t = minAlphaToPassThru(vertices[i], dir, vertices[edge.first], vertices[edge.second]);
            alpha = std::min(t, alpha);
        }
    }

    if(!bAnyPenetration_ccd(alpha))
    {
        return alpha;
    }
    else
    {
        alpha = binarySearch_ccd(0.0f, alpha, 0.1f);
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

float phyScene::repulsiveEnergyVal(float alpha)
{
    repPairlst.clear();
    float sum = 0.0f;
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        for(unsigned int j = 0; j < edges.size(); j ++)
        {
            if(i == edges[j].first || i == edges[j].second)
            {
                continue;
            }
            unsigned int idx0 = edges[j].first;
            unsigned int idx1 = edges[j].second;
            Eigen::Vector2f p = vertices[i];
            p(0) += alpha * searchDir(2 * i);
            p(1) += alpha * searchDir(2 * i + 1);
            Eigen::Vector2f e0 = vertices[idx0];
            e0(0) += alpha * searchDir(2 * idx0);
            e0(1) += alpha * searchDir(2 * idx0 + 1);
            Eigen::Vector2f e1 = vertices[idx1];
            e1(0) += alpha * searchDir(2 * idx1);
            e1(1) += alpha * searchDir(2 * idx1 + 1);
            EPointEdgeDistanceType petype = point_edge_dist_type(p, e0, e1);
            if(petype == EPointEdgeDistanceType::P_E)
            {
                float length = (e0 - e1).norm();
                Eigen::Vector2f e0p = e0 - p;
                Eigen::Vector2f e1p = e1 - p;
                float d = std::abs(e0p.x() * e1p.y() - e0p.y() * e1p.x()) / length;
                if(d < dhat)
                {
                    //sum += contactArea[i] * dhat * kappa / 2 * repulsion(d * d - dmin * dmin, dhat * dhat + 2 * dmin * dhat);
                    sum += contactArea[i] * dhat * kappa / 2 * repulsion(d * d, dhat * dhat) * amplifier;
                    repPairlst.emplace_back(i, j, EPointEdgeDistanceType::P_E);
                }
            }
            else if(petype == EPointEdgeDistanceType::P_E0)
            {
                float dsq = (p - e0).squaredNorm();
                if(dsq < dhat * dhat)
                {
                    //sum += contactArea[i] * dhat * kappa / 2 * repulsion(dsq - dmin * dmin, dhat * dhat + 2 * dmin * dhat);
                    sum += contactArea[i] * dhat * kappa / 2 * repulsion(dsq, dhat * dhat) * amplifier;
                    repPairlst.emplace_back(i, j, EPointEdgeDistanceType::P_E0);
                }
            }
            else if(petype == EPointEdgeDistanceType::P_E1)
            {
                float dsq = (p - e1).squaredNorm();
                if(dsq < dhat * dhat)
                {
                    //sum += contactArea[i] * dhat * kappa / 2 * repulsion(dsq - dmin * dmin, dhat * dhat + 2 * dmin * dhat);
                    sum += contactArea[i] * dhat * kappa / 2 * repulsion(dsq, dhat * dhat) * amplifier;
                    repPairlst.emplace_back(i, j, EPointEdgeDistanceType::P_E1);
                }
            }
            else
            {
                printf("EPointEdgeDistanceType doesn't exist!\n");
            }
        }
    }
    return sum;
}

void phyScene::calcRepulsiveEnergyGradient(float dt)
{
    for(auto repPair : repPairlst)
    {
        float w = contactArea[repPair.pt_idx] * dhat * kappa / 2 * amplifier;
        unsigned int idx = repPair.pt_idx;
        unsigned int idx0 = edges[repPair.edge_idx].first;
        unsigned int idx1 = edges[repPair.edge_idx].second;
        Eigen::Vector2f e0 = vertices[idx0];
        Eigen::Vector2f e1 = vertices[idx1];
        Eigen::Vector2f p = vertices[idx];

        if(repPair.PE_type == EPointEdgeDistanceType::P_E)
        {
            // distance
            float lensq = (e0 - e1).squaredNorm();
            Eigen::Vector2f e0p = e0 - p;
            Eigen::Vector2f e1p = e1 - p;
            float area = std::abs(e0p.x() * e1p.y() - e0p.y() * e1p.x());
            float dsq = area * area / lensq;

            //float grad_f = repulsionFirstDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            float grad_f = repulsionFirstDerivative(dsq, dhat * dhat);

            float grad[6];

            float a = vertices[idx](0);
            float b = vertices[idx](1);
            float c = vertices[idx0](0);
            float d = vertices[idx0](1);
            float e = vertices[idx1](0);
            float f = vertices[idx1](1);
            point_line_distance_gradient_2D(a,b,c,d,e,f,grad);
            energyGradient(2 * repPair.pt_idx) += dt * dt * grad[0] * w * grad_f;
            energyGradient(2 * repPair.pt_idx + 1) += dt * dt * grad[1] * w * grad_f;
            energyGradient(2 * idx0) += dt * dt * grad[2] * w * grad_f;
            energyGradient(2 * idx0 + 1) += dt * dt * grad[3] * w * grad_f;
            energyGradient(2 * idx1) += dt * dt * grad[4] * w * grad_f;
            energyGradient(2 * idx1 + 1) += dt * dt * grad[5] * w * grad_f;
        }
        else if(repPair.PE_type == EPointEdgeDistanceType::P_E0)
        {
            float dsq = (p - e0).squaredNorm();
            //float grad_f = repulsionFirstDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            float grad_f = repulsionFirstDerivative(dsq, dhat * dhat);
            Eigen::Vector4f grad = point_point_dist_gradient(vertices[idx], vertices[idx0]);
            energyGradient(2 * idx) += dt * dt * grad[0] * w * grad_f;
            energyGradient(2 * idx + 1) += dt * dt * grad[1] * w * grad_f;
            energyGradient(2 * idx0) += dt * dt * grad[2] * w * grad_f;
            energyGradient(2 * idx0 + 1) += dt * dt * grad[3] * w * grad_f;
        }
        else if(repPair.PE_type == EPointEdgeDistanceType::P_E1)
        {
            float dsq = (p - e1).squaredNorm();
            //float grad_f = repulsionFirstDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            float grad_f = repulsionFirstDerivative(dsq, dhat * dhat);
            Eigen::Vector4f grad = point_point_dist_gradient(vertices[idx], vertices[idx1]);
            energyGradient(2 * idx) += dt * dt * grad[0] * w * grad_f;
            energyGradient(2 * idx + 1) += dt * dt * grad[1] * w * grad_f;
            energyGradient(2 * idx1) += dt * dt * grad[2] * w * grad_f;
            energyGradient(2 * idx1 + 1) += dt * dt * grad[3] * w * grad_f;
        }
        else
        {
            printf("EPointEdgeDistanceType doesn't exist!\n");
        }
    }
}

void phyScene::calcRepulsiveEnergyHessian(float dt)
{
    for(auto repPair : repPairlst)
    {
        float w = contactArea[repPair.pt_idx] * dhat * kappa / 2 * amplifier;
        unsigned int idx = repPair.pt_idx;
        unsigned int idx0 = edges[repPair.edge_idx].first;
        unsigned int idx1 = edges[repPair.edge_idx].second;
        Eigen::Vector2f e0 = vertices[idx0];
        Eigen::Vector2f e1 = vertices[idx1];
        Eigen::Vector2f p = vertices[idx];

        if(repPair.PE_type == EPointEdgeDistanceType::P_E)
        {
            float lensq = (e0 - e1).squaredNorm();
            Eigen::Vector2f e0p = e0 - p;
            Eigen::Vector2f e1p = e1 - p;
            float area = std::abs(e0p.x() * e1p.y() - e0p.y() * e1p.x());
            float dsq = area * area / lensq;

            //float grad_f = repulsionFirstDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            //float hess_f = repulsionSecondDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            float grad_f = repulsionFirstDerivative(dsq, dhat * dhat);
            float hess_f = repulsionSecondDerivative(dsq, dhat * dhat);

            // \delta d
            float grad[6];
            // \delta^2 d
            float hess[36];

            float a = vertices[idx](0);
            float b = vertices[idx](1);
            float c = vertices[idx0](0);
            float d = vertices[idx0](1);
            float e = vertices[idx1](0);
            float f = vertices[idx1](1);
            point_line_distance_gradient_2D(a,b,c,d,e,f,grad);
            point_line_distance_hessian_2D(a,b,c,d,e,f,hess);

            float ddT[36];
            for(int i = 0; i < 6; i++)
            {
                for(int j = 0; j < 6; j++)
                {
                    ddT[6 * i + j] = grad[i] * grad[j] * hess_f * w;
                }
            }

            // assemble hessian
            for(int i = 0; i < 36; i++)
            {
                hess[i] = hess[i] * grad_f * w + ddT[i];
            }

            spdProjection6x6(hess);

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    energyHessian.emplace_back(2 * idx + i, 2 * idx + j, hess[i * 6 + j] * dt * dt);
                }
            }

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    energyHessian.emplace_back(2 * idx0 + i, 2 * idx0 + j, hess[14 + i * 6 + j] * dt * dt);
                }
            }

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    energyHessian.emplace_back(2 * idx1 + i, 2 * idx1 + j, hess[28 + i * 6 + j] * dt * dt);
                }
            }

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    energyHessian.emplace_back(2 * idx + i, 2 * idx0 + j, hess[2 + i * 6 + j] * dt * dt);
                    energyHessian.emplace_back(2 * idx0 + j, 2 * idx + i, hess[2 + i * 6 + j] * dt * dt);
                }
            }

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    energyHessian.emplace_back(2 * idx + i, 2 * idx1 + j, hess[4 + i * 6 + j] * dt * dt);
                    energyHessian.emplace_back(2 * idx1 + j, 2 * idx + i, hess[4 + i * 6 + j] * dt * dt);
                }
            }

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    energyHessian.emplace_back(2 * idx0 + i, 2 * idx1 + j, hess[16 + i * 6 + j] * dt * dt);
                    energyHessian.emplace_back(2 * idx1 + j, 2 * idx0 + i, hess[16 + i * 6 + j] * dt * dt);
                }
            }
        }
        else if(repPair.PE_type == EPointEdgeDistanceType::P_E0)
        {
            float dsq = (p - e0).squaredNorm();
            //float grad_f = repulsionFirstDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            //float hess_f = repulsionSecondDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            float grad_f = repulsionFirstDerivative(dsq, dhat * dhat);
            float hess_f = repulsionSecondDerivative(dsq, dhat * dhat);
            Eigen::Vector4f grad = point_point_dist_gradient(vertices[idx], vertices[idx0]);
            Eigen::Matrix4f hess = grad * grad.transpose();
            hess = w * hess_f * hess;
            float factor = grad_f * w;
            hess(0, 0) += 2 * factor;
            hess(1, 1) += 2 * factor;
            hess(2, 2) += 2 * factor;
            hess(3, 3) += 2 * factor;
            hess(2, 0) -= 2 * factor;
            hess(3, 1) -= 2 * factor;
            hess(0, 2) -= 2 * factor;
            hess(1, 3) -= 2 * factor;

            hess = spdProjection(hess);

            energyHessian.emplace_back(2 * idx, 2 * idx, hess(0, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx + 1, hess(1, 1) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx, hess(1, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx, 2 * idx + 1, hess(0, 1) * dt * dt);

            energyHessian.emplace_back(2 * idx0, 2 * idx0, hess(2, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx0 + 1, 2 * idx0 + 1, hess(3, 3) * dt * dt);
            energyHessian.emplace_back(2 * idx0 + 1, 2 * idx0, hess(3, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx0, 2 * idx0 + 1, hess(2, 3) * dt * dt);

            energyHessian.emplace_back(2 * idx, 2 * idx0, hess(0, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx0 + 1, hess(1, 3) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx0, hess(1, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx, 2 * idx0 + 1, hess(0, 3) * dt * dt);

            energyHessian.emplace_back(2 * idx0, 2 * idx, hess(2, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx0 + 1, 2 * idx + 1, hess(3, 1) * dt * dt);
            energyHessian.emplace_back(2 * idx0 + 1, 2 * idx, hess(3, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx0, 2 * idx + 1, hess(2, 1) * dt * dt);

        }
        else if(repPair.PE_type == EPointEdgeDistanceType::P_E1)
        {
            float dsq = (p - e1).squaredNorm();
            //float grad_f = repulsionFirstDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            //float hess_f = repulsionSecondDerivative(dsq - dmin * dmin, 2 * dmin * dhat + dhat * dhat);
            float grad_f = repulsionFirstDerivative(dsq, dhat * dhat);
            float hess_f = repulsionSecondDerivative(dsq, dhat * dhat);
            Eigen::Vector4f grad = point_point_dist_gradient(vertices[idx], vertices[idx1]);
            Eigen::Matrix4f hess = grad * grad.transpose();
            hess = w * hess_f * hess;
            float factor = grad_f * w;
            hess(0, 0) += 2 * factor;
            hess(1, 1) += 2 * factor;
            hess(2, 2) += 2 * factor;
            hess(3, 3) += 2 * factor;
            hess(2, 0) -= 2 * factor;
            hess(3, 1) -= 2 * factor;
            hess(0, 2) -= 2 * factor;
            hess(1, 3) -= 2 * factor;

            hess = spdProjection(hess);

            energyHessian.emplace_back(2 * idx, 2 * idx, hess(0, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx + 1, hess(1, 1) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx, hess(1, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx, 2 * idx + 1, hess(0, 1) * dt * dt);

            energyHessian.emplace_back(2 * idx1, 2 * idx1, hess(2, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx1 + 1, 2 * idx1 + 1, hess(3, 3) * dt * dt);
            energyHessian.emplace_back(2 * idx1 + 1, 2 * idx1, hess(3, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx1, 2 * idx1 + 1, hess(2, 3) * dt * dt);

            energyHessian.emplace_back(2 * idx, 2 * idx1, hess(0, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx1 + 1, hess(1, 3) * dt * dt);
            energyHessian.emplace_back(2 * idx + 1, 2 * idx1, hess(1, 2) * dt * dt);
            energyHessian.emplace_back(2 * idx, 2 * idx1 + 1, hess(0, 3) * dt * dt);

            energyHessian.emplace_back(2 * idx1, 2 * idx, hess(2, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx1 + 1, 2 * idx + 1, hess(3, 1) * dt * dt);
            energyHessian.emplace_back(2 * idx1 + 1, 2 * idx, hess(3, 0) * dt * dt);
            energyHessian.emplace_back(2 * idx1, 2 * idx + 1, hess(2, 1) * dt * dt);
        }
        else
        {
            printf("EPointEdgeDistanceType doesn't exist!\n");
        }
    }
}


bool phyScene::b2TrianglesIntersect(unsigned int idx1, unsigned int idx2)
{
    for(unsigned int i = 0; i < 3; i++)
    {
        for(unsigned int j = 0; j < 3; j++)
        {
            if(eidx[idx1](i) == eidx[idx2](j))
                return false;
        }
    }

    for(unsigned int i = 0; i < 3; i++)
    {
        Eigen::Vector2f v = vertices[eidx[idx1](i)];
        Eigen::Vector2f v0 = vertices[eidx[idx2](0)];
        Eigen::Vector2f v1 = vertices[eidx[idx2](1)];
        Eigen::Vector2f v2 = vertices[eidx[idx2](2)];

        float c1 = crossProduct(v0, v1, v);
        float c2 = crossProduct(v1, v2, v);
        float c3 = crossProduct(v2, v0, v);

        bool bSameSign = (c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0);
        if(bSameSign)
        {
            return true;
        }
    }
    return false;
}

bool phyScene::bAnyPenetration()
{
    unsigned int sz = eidx.size();
    for(unsigned int i = 0; i < sz; i++)
    {
        for(unsigned int j = i + 1; j < sz; j++)
        {
            if(b2TrianglesIntersect(i, j))
            {
                return true;
            }
        }
    }
    return false;
}

bool phyScene::b2TrianglesIntersect_ccd(unsigned int idx1, unsigned int idx2, float alpha)
{
    for(unsigned int i = 0; i < 3; i++)
    {
        for(unsigned int j = 0; j < 3; j++)
        {
            if(eidx[idx1](i) == eidx[idx2](j))
                return false;
        }
    }

    for(unsigned int i = 0; i < 3; i++)
    {
        unsigned int vidx = eidx[idx1](i);
        unsigned int vidx0 = eidx[idx2](0);
        unsigned int vidx1 = eidx[idx2](1);
        unsigned int vidx2 = eidx[idx2](2);
        Eigen::Vector2f v = vertices[vidx];
        Eigen::Vector2f v0 = vertices[vidx0];
        Eigen::Vector2f v1 = vertices[vidx1];
        Eigen::Vector2f v2 = vertices[vidx2];
        v(0) += alpha * searchDir(2 * vidx);
        v(1) += alpha * searchDir(2 * vidx + 1);
        v0(0) += alpha * searchDir(2 * vidx0);
        v0(1) += alpha * searchDir(2 * vidx0 + 1);
        v1(0) += alpha * searchDir(2 * vidx1);
        v1(1) += alpha * searchDir(2 * vidx1 + 1);
        v2(0) += alpha * searchDir(2 * vidx2);
        v2(1) += alpha * searchDir(2 * vidx2 + 1);

        float c1 = crossProduct(v0, v1, v);
        float c2 = crossProduct(v1, v2, v);
        float c3 = crossProduct(v2, v0, v);

        bool bSameSign = (c1 >= 0 && c2 >= 0 && c3 >= 0) || (c1 <= 0 && c2 <= 0 && c3 <= 0);
        if(bSameSign)
        {
            return true;
        }
    }
    return false;
}

bool phyScene::bAnyPenetration_ccd(float alpha)
{
    unsigned int sz = eidx.size();
    for(unsigned int i = 0; i < sz; i++)
    {
        for(unsigned int j = i + 1; j < sz; j++)
        {
            if(b2TrianglesIntersect_ccd(i, j, alpha))
            {
                return true;
            }
        }
    }
    return false;
}

float phyScene::binarySearch_ccd(float low, float high, float precision)
{
    if(high <= low)
    {
        return low;
    }

    float mid;
    while(high - low > precision)
    {
        mid = low + (high - low) / 2;
        bool bPnt = bAnyPenetration_ccd(mid);
        if(bPnt)
        {
            high = mid;
        }
        else
        {
            low = mid;
        }
    }
    return (low + high) / 2;
}