//
// Created by Xiaopeng Zhang on 3/10/24.
//

#ifndef IPC2DSIM_OBJECT_H
#define IPC2DSIM_OBJECT_H

#include "macros.h"
#include <glad.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include <vector>
#include "shader.h"
#include "camera.h"
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
    unsigned int VAO;
    unsigned int VBO;

    shader shaderInst;

    // object type
    EObjectType objectType;

    Object(const std::string &filename, EObjectType type, shader shaderInstance);

    void Draw(Camera& cam);

    void updateVertexBuffer();

private:
    void setupObject();

};






#endif //IPC2DSIM_OBJECT_H
