//
// Created by Xiaopeng Zhang on 3/10/24.
//

#ifndef IPC2DSIM_OBJECT_H
#define IPC2DSIM_OBJECT_H

#include "macros.h"
#include <glad.h>
#include <vector>
#include "shader.h"
#include "camera.h"
#include <cmath>
//#include "utils.h"

struct elementIndex
{
public:
    unsigned int indices[3]{};

    elementIndex(){ indices[0] = 0; indices[1] = 0; indices[2] = 0;}

    elementIndex(unsigned int x, unsigned int y, unsigned int z)
    {
        indices[0] = x;
        indices[1] = y;
        indices[2] = z;
    }

};

enum class EObjectType
{
    STATIC,
    DYNAMIC
};

class Object
{
public:

    std::vector<Eigen::Vector2f> vertices;
    std::vector<elementIndex> elementIndices;
    std::vector<std::pair<unsigned int, unsigned int>> edges;
    unsigned int VAO = 0;
    unsigned int VBO = 0;

    shader shaderInst;

    // object type
    EObjectType objectType = EObjectType::STATIC;

    Object() = default;
    Object(const std::string &filename, EObjectType type, shader shaderInstance);

    void Draw(Camera& cam);

    void updateVertexBuffer();

    void stretch(float factor);

private:
    void setupObject();

};






#endif //IPC2DSIM_OBJECT_H
