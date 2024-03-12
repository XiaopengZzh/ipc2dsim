//
// Created by Xiaopeng Zhang on 3/10/24.
//

#include "Object.h"
#include <fstream>


Object::Object(const std::string &filename, EObjectType type, shader shaderInstance)
{
    //location = glm::vec3(0.0f);
    //rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    objectType = type;
    shaderInst = shaderInstance;
    VAO = 0;
    VBO = 0;

    std::ifstream in(filename);

    int ptNum;
    in >> ptNum;

    for(int idx = 0; idx < ptNum; idx++)
    {
        float x, y, z;
        in >> x >> y >> z;
        vertices.emplace_back(x, y, z);
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

    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, elementIndices.size() * sizeof(elementIndex), elementIndices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
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

    /*
    glm::mat4 model = glm::translate(glm::mat4(1.0f), location);

    if(rotation.w != 1)
    {
        model = glm::rotate(model, 2 * glm::acos(rotation.w), glm::vec3( rotation.x, rotation.y, rotation.z ));
    }

    shaderInst.setMat4("model", model);
     */

    glBindVertexArray(VAO);

    glDrawElements(GL_TRIANGLES, vertices.size(), GL_UNSIGNED_INT, 0);
    //glDrawArrays(GL_TRIANGLES, 0, vertices.size());
}

void Object::updateVertexBuffer()
{
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(glm::vec3), vertices.data());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}