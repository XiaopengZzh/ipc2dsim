//
// Created by Xiaopeng Zhang on 3/11/24.
//

#ifndef IPC2DSIM_WORLD_H
#define IPC2DSIM_WORLD_H

#include <memory>
#include <vector>
#include <iostream>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "Object.h"
#include "shader.h"

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

    void Draw(Camera &cam);

    std::vector<Object> objList;


private:
    world();
};

#endif //IPC2DSIM_WORLD_H
