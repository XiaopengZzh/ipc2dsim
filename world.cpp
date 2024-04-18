//
// Created by Xiaopeng Zhang on 3/11/24.
//

#include "world.h"
#include "eigen-3.4.0/Eigen/Core"


void append(std::vector<std::pair<unsigned int, unsigned int>>& edges,
            const std::vector<std::pair<unsigned int, unsigned int>>& edgesToAdd)
{
    unsigned int baseNum = edges.size();
    for(auto edge : edgesToAdd)
    {
        edges.emplace_back(edge.first + baseNum, edge.second + baseNum);
    }
}

void appendEBO(std::vector<Eigen::Vector3i>& eidx, const std::vector<elementIndex>& eidxToAdd, unsigned int baseNum)
{
    for(auto e : eidxToAdd)
    {
        eidx.emplace_back(e.indices[0] + baseNum, e.indices[1] + baseNum, e.indices[2] + baseNum);
    }
}



world::world()
{
    std::cout << "World is created." << std::endl;
    shader barrierShader("../shaders/vertex.vs", "../shaders/barrier.fs");
    yBarrier = Object("../model/yBarrier.xyz", EObjectType::STATIC, barrierShader);
}

void world::CreateObject(const std::string &filename, EObjectType type, shader shaderInstance)
{
    objList.emplace_back(filename, type, shaderInstance);
}

void world::initialStretch(float factor)
{
    for(auto& obj : objList)
    {
        for(auto& vert : obj.vertices)
        {
            vert[0] *= factor;
        }
    }
}


void world::physRegistration()
{
    unsigned int totalNumVerts = 0;

    float temp_mu = 0.5f * E / (1 + nu);
    float temp_lam = E * nu / ((1 + nu) * (1 - 2 * nu));

    for(auto& obj : objList)
    {
        for(auto edge : obj.edges)
        {
            Eigen::Vector2f diff = obj.vertices[edge.first] - obj.vertices[edge.second];
            physics.squaredRestLengths.emplace_back(diff.dot(diff));
        }

        //initialStretch(1.4f);
        //obj.stretch(1.4f);

        appendEBO(physics.eidx, obj.elementIndices, physics.vertices.size());

        physics.vertices.insert(physics.vertices.end(), obj.vertices.begin(), obj.vertices.end());
        append(physics.edges, obj.edges);

        for(unsigned int i = 0; i < obj.elementIndices.size(); i++)
        {
            unsigned int e0 = obj.elementIndices[i].indices[0];
            unsigned int e1 = obj.elementIndices[i].indices[1];
            unsigned int e2 = obj.elementIndices[i].indices[2];
            Eigen::Matrix2f TB;
            //TB << obj.vertices[e1].x() - obj.vertices[e0].x(), obj.vertices[e2].x() - obj.vertices[e0].x(), obj.vertices[e1].y() - obj.vertices[e0].y(), obj.vertices[e2].y() - obj.vertices[e0].y();
            TB.col(0) = obj.vertices[e1] - obj.vertices[e0];
            TB.col(1) = obj.vertices[e2] - obj.vertices[e0];
            physics.vol.emplace_back(TB.determinant() / 2.0f);
            physics.IB.emplace_back(TB.inverse());
            physics.lams.emplace_back(temp_lam);
            physics.mus.emplace_back(temp_mu);
        }

        totalNumVerts += obj.vertices.size();
        for(unsigned int i = 0; i < obj.vertices.size(); i++)
        {
            physics.velocities.emplace_back(0.0f, 0.0f);
            physics.verts_tilde.emplace_back(0.0f, 0.0f);
            //physics.energyGradient.emplace_back(0.0f);
            //physics.energyGradient.emplace_back(0.0f);
            physics.mass.emplace_back(40.0f);
            physics.contactArea.emplace_back(0.25f);
        }
    }
    physics.energyGradient = Eigen::VectorXf::Zero(2 * totalNumVerts);
    physics.searchDir = Eigen::VectorXf::Zero(2 * totalNumVerts);
}

void world::broadcastLocations()
{
    unsigned int i = 0;
    for(auto& obj : objList)
    {
        for(auto& vert : obj.vertices)
        {
            vert = physics.vertices[i];
            i++;
        }
        obj.updateVertexBuffer();
    }
}


void world::Draw(Camera &cam)
{
    for(auto& obj : objList)
    {
        obj.Draw(cam);
    }
    yBarrier.Draw(cam);
}

void world::simulate(float dt)
{
    physics.oneTimestepImpl(dt);
    broadcastLocations();
}

void world::clear()
{
    objList.clear();
    physics = phyScene();
    // bRoundbouned?
}