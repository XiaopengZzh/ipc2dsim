//
// Created by Xiaopeng Zhang on 3/10/24.
//

#ifndef IPC2DSIM_MACROS_H
#define IPC2DSIM_MACROS_H

const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

#define ZNEAR 0.1f
#define ZFAR 1000.0f

// set this to 0 if output obj file
#define DISABLE_RENDER 1

#if DISABLE_RENDER
#define RENDER_ENABLE 0
#define OUTPUT_OBJ 1
#else
#define RENDER_ENABLE 1
#define OUTPUT_OBJ 0
#endif

#endif //IPC2DSIM_MACROS_H
