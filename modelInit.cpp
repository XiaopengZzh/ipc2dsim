//
// Created by Xiaopeng Zhang on 3/11/24.
//

#include "modelInit.h"

void modelInit()
{
    printf("initializing models...\n");
    std::shared_ptr<world> world = world::GetWorldInstance();

    shader simshader("../shaders/vertex.vs", "../shaders/fragment.fs");
    simshader.use();

    world->CreateObject("../model/1tri1cube.xyz", EObjectType::DYNAMIC, simshader);
    //world->CreateObject("../model/tetrahedron.xyz", EObjectType::STATIC, simshader);
}
