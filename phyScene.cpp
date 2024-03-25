//
// Created by Xiaopeng Zhang on 3/23/24.
//

#include "phyScene.h"
#include "../eigen-3.4.0/Eigen/Dense"

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



void phyScene::calcVertsTilde(float dt)
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        verts_tilde[i] = vertices[i] - dt * velocities[i];
    }
}

float phyScene::calcEnergy(float dt, const Eigen::Vector2f& offset)
{
    return inertiaEnergyVal(offset) + dt * dt * springEnergyVal(offset);
}




void phyScene::oneTimestepImpl(float dt)
{
    energyGradient.assign(energyGradient.size(), 0.0f);
    energyHessian.loc_i.clear();
    energyHessian.loc_j.clear();
    energyHessian.vals.clear();

    calcVertsTilde(dt);
}






// todo : integrating mass, spring stiffness
float phyScene::inertiaEnergyVal(const Eigen::Vector2f& offset)
{
    float sum = 0.0f;
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector2f diff = vertices[i] - verts_tilde[i] + offset;
        sum += 0.5f * diff.dot(diff);
    }
    return sum;
}

void phyScene::calcInertiaEnergyGradient()
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        Eigen::Vector2f diff = vertices[i] - verts_tilde[i];
        energyGradient[2 * i] += diff.x();
        energyGradient[2 * i + 1] += diff.y();
    }
}

void phyScene::calcInertiaEnergyHessian()
{
    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        energyHessian.loc_i.push_back(2 * i);
        energyHessian.loc_j.push_back(2 * i);
        energyHessian.vals.push_back(1.0f);
        energyHessian.loc_i.push_back(2 * i + 1);
        energyHessian.loc_j.push_back(2 * i + 1);
        energyHessian.vals.push_back(1.0f);
    }
}

//=====================================

float phyScene::springEnergyVal(const Eigen::Vector2f& offset)
{
    float sum = 0.0f;
    for(unsigned int i = 0; i < edges.size(); i++)
    {
        Eigen::Vector2f diff = vertices[edges[i].first] - vertices[edges[i].second] + offset;
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
        Eigen::Vector2f diff = vertices[edges[i].first] - vertices[edges[i].second];
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
                energyHessian.loc_i.push_back(edges[i].first * 2 + r);
                energyHessian.loc_j.push_back(edges[i].first * 2 + c);
                energyHessian.vals.push_back(H_local(r, c));

                energyHessian.loc_i.push_back(edges[i].first * 2 + r);
                energyHessian.loc_j.push_back(edges[i].second * 2 + c);
                energyHessian.vals.push_back(H_local(r, c + 2));

                energyHessian.loc_i.push_back(edges[i].second * 2 + r);
                energyHessian.loc_j.push_back(edges[i].first * 2 + c);
                energyHessian.vals.push_back(H_local(r + 2, c));

                energyHessian.loc_i.push_back(edges[i].second * 2 + r);
                energyHessian.loc_j.push_back(edges[i].second * 2 + c);
                energyHessian.vals.push_back(H_local(r + 2, c + 2));
            }
        }
    }
}