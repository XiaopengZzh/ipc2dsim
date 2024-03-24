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
}

void world::CreateObject(const std::string &filename, EObjectType type, shader shaderInstance)
{
    objList.emplace_back(filename, type, shaderInstance);
}

void world::physRegistration()
{
    for(auto obj : objList)
    {
        physics.vertices.insert(physics.vertices.end(), obj.vertices.begin(), obj.vertices.end());
        append(physics.edges, obj.edges);
    }
}


void world::Draw(Camera &cam)
{
    for(auto& obj : objList)
    {
        obj.Draw(cam);
    }
}

void world::simulate(float dt)
{

}