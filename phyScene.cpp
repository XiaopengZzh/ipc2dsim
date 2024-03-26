//
// Created by Xiaopeng Zhang on 3/23/24.
//

#include "phyScene.h"
#include "eigen-3.4.0/Eigen/Dense"

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
    return inertiaEnergyVal(alpha) + dt * dt * springEnergyVal(alpha);
}

void phyScene::calcEnergyGradient(float dt)
{
    calcInertiaEnergyGradient();
    calcSpringEnergyGradient(dt);
}

void phyScene::calcEnergyHessian(float dt)
{
    calcInertiaEnergyHessian();
    calcSpringEnergyHessian(dt);
}

void phyScene::calcSearchDir()
{
    unsigned int dim = 2 * vertices.size();
    Eigen::SparseMatrix<float> hessian(dim, dim);
    hessian.setFromTriplets(energyHessian.begin(), energyHessian.end());
    Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
    //solver
    solver.analyzePattern(hessian);
    solver.factorize(hessian);
    searchDir = solver.solve(-energyGradient);
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

        float a = calcEnergy(dt, alpha);

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
        sum += 0.5f * diff.dot(diff);
    }
    return 40.0f * sum;
}

void phyScene::calcInertiaEnergyGradient()
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector2f diff = vertices[i] - verts_tilde[i];
        energyGradient[2 * i] += diff.x() * 40.0f;
        energyGradient[2 * i + 1] += diff.y() * 40.0f;
    }
}

void phyScene::calcInertiaEnergyHessian()
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        energyHessian.emplace_back(2 * i, 2 * i, 40.0f);
        energyHessian.emplace_back(2 * i + 1, 2 * i + 1, 40.0f);
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