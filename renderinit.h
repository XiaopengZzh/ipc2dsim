//
// Created by Xiaopeng Zhang on 3/10/24.
//

#ifndef IPC2DSIM_RENDERINIT_H
#define IPC2DSIM_RENDERINIT_H

#include <glad.h>
#include <glfw3.h>
#include "macros.h"
#include <iostream>
#include "camera.h"

GLFWwindow* renderInit();
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
//void mouse_callback(GLFWwindow* window, double xposIn, double ypos);
//void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

#endif //IPC2DSIM_RENDERINIT_H
