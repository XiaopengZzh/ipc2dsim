//
// Created by Xiaopeng Zhang on 3/10/24.
//

#include "Object.h"
#include <fstream>


Object::Object(const std::string &filename, EObjectType type, shader shaderInstance)
{
    objectType = type;
    shaderInst = shaderInstance;
    VAO = 0;
    VBO = 0;

    std::ifstream in(filename);

    int ptNum;
    in >> ptNum;

    for(int idx = 0; idx < ptNum; idx++)
    {
        float x, y;
        in >> x >> y;
        vertices.emplace_back(x, y);
    }

    int elementBufferNum;
    in >> elementBufferNum;

    for(int idx = 0; idx < elementBufferNum; idx++)
    {
        elementIndex buf;
        in >> buf.indices[0] >> buf.indices[1] >> buf.indices[2];
        elementIndices.push_back(buf);
    }

    setupObject();
}


void Object::setupObject()
{
    unsigned int EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec2), vertices.data(), GL_DYNAMIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, elementIndices.size() * sizeof(elementIndex), elementIndices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    // do NOT unbind the EBO while a VAO is active
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void Object::Draw(Camera &cam)
{
    if(shaderInst.ID == 0)
        return;

    shaderInst.use();
    //shaderInst.setVec3("viewPos", cam.Position);
    shaderInst.setVec3("viewPos", cam.Position);
    glm::mat4 projection = glm::perspective(glm::radians(cam.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, ZNEAR, ZFAR);
    shaderInst.setMat4("projection", projection);
    glm::mat4 view = cam.GetViewMatrix();
    shaderInst.setMat4("view", view);

    glBindVertexArray(VAO);

    glDrawElements(GL_TRIANGLES, vertices.size(), GL_UNSIGNED_INT, 0);
}

void Object::updateVertexBuffer()
{
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(glm::vec2), vertices.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}