//
// Created by Xiaopeng Zhang on 3/11/24.
//

#include "world.h"

void append(std::vector<std::pair<unsigned int, unsigned int>>& edges,
            const std::vector<std::pair<unsigned int, unsigned int>>& edgesToAdd)
{
    unsigned int baseNum = edges.size();
    for(auto edge : edgesToAdd)
    {
        edges.emplace_back(edge.first + baseNum, edge.second + baseNum);
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
    for(auto& obj : objList)
    {
        for(auto edge : obj.edges)
        {
            Eigen::Vector2f diff = obj.vertices[edge.first] - obj.vertices[edge.second];
            physics.squaredRestLengths.emplace_back(diff.dot(diff));
        }

        //initialStretch(1.4f);
        obj.stretch(1.4f);

        physics.vertices.insert(physics.vertices.end(), obj.vertices.begin(), obj.vertices.end());
        append(physics.edges, obj.edges);

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