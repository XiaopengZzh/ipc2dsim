//
// Created by Xiaopeng Zhang on 3/10/24.
//

#ifndef IPC2DSIM_SHADER_H
#define IPC2DSIM_SHADER_H

#include <glad.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include "Eigen/Core"

class shader
{
public:

    unsigned int ID;

    shader();

    shader(const char* vertexPath, const char* fragmentPath);

    //activate the shader
    void use() const;

    // utility uniform functions
    void setBool(const std::string &name, bool value) const;
    void setInt(const std::string &name, int value) const;
    void setFloat(const std::string &name, float value) const;
    void setMat4(const std::string &name, const Eigen::Matrix4f &mat) const;
    void setVec3(const std::string &name, const Eigen::Vector3f &value) const;
    void setVec3(const std::string &name, float x, float y, float z) const;
    
};



#endif //IPC2DSIM_SHADER_H
