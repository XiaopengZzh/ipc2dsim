//
// Created by Xiaopeng Zhang on 3/11/24.
//

#ifndef IPC2DSIM_MODELINIT_H
#define IPC2DSIM_MODELINIT_H
#include "macros.h"
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "Object.h"
#include "world.h"
#include "shader.h"
#include <fstream>
#include <cstdio>

void modelInit();

void modelInit_utest(const std::string &filename);

#endif //IPC2DSIM_MODELINIT_H
