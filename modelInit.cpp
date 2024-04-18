//
// Created by Xiaopeng Zhang on 3/11/24.
//

#include "modelInit.h"
#include <catch2/catch_all.hpp>

extern bool bUnitTest;

void modelInit()
{
    printf("initializing models...\n");
    std::shared_ptr<world> world = world::GetWorldInstance();

    shader simshader("../shaders/vertex.vs", "../shaders/fragment.fs");
    simshader.use();

    if(!bUnitTest)
    {
        world->CreateObject("../model/bent.xyz", EObjectType::DYNAMIC, simshader);
    }
    //world->CreateObject("../model/tetrahedron.xyz", EObjectType::STATIC, simshader);

    /*
    if(bUnitTest)
    {
        SECTION("scenario : 1 triangle, 1 cube")
        {
            world->CreateObject("../model/1tri1cube.xyz", EObjectType::DYNAMIC, simshader);
        }


        SECTION("scenario : 2 cubes")
        {
            world->CreateObject("../model/2cubes.xyz", EObjectType::DYNAMIC, simshader);
        }

        SECTION("scenario : 2 triangles")
        {
            world->CreateObject("../model/2triangles.xyz", EObjectType::DYNAMIC, simshader);
        }
    }
    */
}

void modelInit_utest(const std::string &filename)
{
    if(!bUnitTest)
        return;
    std::shared_ptr<world> world = world::GetWorldInstance();

    shader simshader("../shaders/vertex.vs", "../shaders/fragment.fs");
    simshader.use();
    world->CreateObject(filename, EObjectType::DYNAMIC, simshader);
}

