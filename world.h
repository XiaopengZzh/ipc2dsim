//
// Created by Xiaopeng Zhang on 3/11/24.
//

#ifndef IPC2DSIM_WORLD_H
#define IPC2DSIM_WORLD_H

#include "Object.h"
#include <memory>
#include <vector>
#include <iostream>
#include "phyScene.h"

class world
{
public:

    static std::shared_ptr<world> GetWorldInstance()
    {
        static std::shared_ptr<world> instance(new world);

        return instance;
    }

    world(const world&) = delete;
    world& operator=(const world&) = delete;

    void CreateObject(const std::string &filename, EObjectType type, shader shaderInstance);

    void physRegistration();
    void broadcastLocations();
    void initialStretch(float factor);


    void Draw(Camera &cam);

    void simulate(float dt);

    std::vector<Object> objList;
    phyScene physics;

    Object yBarrier;

private:
    world();
};

#endif //IPC2DSIM_WORLD_H
