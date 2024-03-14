//
// Created by Xiaopeng Zhang on 3/10/24.
//

#include "shader.h"

shader::shader()
{
    ID = 0;
}

shader::shader(const char *vertexPath, const char *fragmentPath)
{
    std::string vertexCode;
    std::string fragmentCode;
    std::ifstream vShaderFile;
    std::ifstream fShaderFile;

    vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);

    try
    {
        vShaderFile.open(vertexPath);
        fShaderFile.open(fragmentPath);
        std::stringstream vShaderStream, fShaderStream;

        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();

        vShaderFile.close();
        fShaderFile.close();

        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();

    }
    catch(std::ifstream::failure &e)
    {
        std::cout << "ERROR : shader file not successfully read" << std::endl;
    }

    const char* vShaderCode = vertexCode.c_str();
    const char* fShaderCode = fragmentCode.c_str();

    // Similar to VBO buffer object, we assign a name for vertex shader and fragment shader
    GLuint vertex, fragment;
    int CompiledSuccess;
    char infoLog[512];

    // Create a vertex shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderCode, nullptr);

    glCompileShader(vertex);

    // check if vertex shader is successfully compiled
    glGetShaderiv(vertex, GL_COMPILE_STATUS, &CompiledSuccess);
    if(!CompiledSuccess)
    {
        glGetShaderInfoLog(vertex, 512, nullptr, infoLog);
        std::cout << "ERROR : vertex shader compilation failed\n" << infoLog << std::endl;
    }

    // Create a fragment shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderCode, nullptr);

    glCompileShader(fragment);
    // Check if fragment shader is successfully compiled.
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &CompiledSuccess);
    if(!CompiledSuccess)
    {
        glGetShaderInfoLog(fragment, 512, nullptr, infoLog);
        std::cout << "ERROR : fragment shader compilation failed\n" << infoLog << std::endl;
    }

    ID = glCreateProgram();

    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);

    glLinkProgram(ID);
    glGetProgramiv(ID, GL_LINK_STATUS, &CompiledSuccess);
    if(!CompiledSuccess)
    {
        glGetShaderInfoLog(ID, 512, nullptr, infoLog);
        std::cout << "ERROR : shader program linking failed\n" << infoLog << std::endl;
    }

    glDeleteShader(vertex);
    glDeleteShader(fragment);
}

// activate shader program. Every shader and render call after glUseProgram will now use this program object.
void shader::use() const
{
    glUseProgram(ID);
}

void shader::setBool(const std::string &name, bool value) const
{
    glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
}

void shader::setInt(const std::string &name, int value) const
{
    glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
}

void shader::setFloat(const std::string &name, float value) const
{
    glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
}

void shader::setMat4(const std::string &name, const Eigen::Matrix4f &mat) const
{
    glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, mat.data());
}

void shader::setVec3(const std::string &name, const Eigen::Vector3f &value) const
{
    glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, value.data());
}

void shader::setVec3(const std::string &name, float x, float y, float z) const
{
    glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
}