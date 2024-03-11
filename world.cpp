//
// Created by Xiaopeng Zhang on 3/11/24.
//

#include "world.h"


world::world()
{
    std::cout << "World is created." << std::endl;
}

void world::CreateObject(const std::string &filename, EObjectType type, shader shaderInstance)
{
    objList.emplace_back(filename, type, shaderInstance);
}

void world::Draw(Camera &cam)
{
    for(auto& obj : objList)
    {
        obj.Draw(cam);
    }
}