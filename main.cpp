
#include "renderinit.h"
#include "modelInit.h"
#include "world.h"

#include "eigen-3.4.0/Eigen/Dense"
#include <chrono>

extern Camera camera;
extern float deltaTime, lastFrame;

int main(int argc, char* argv[])
{
    printf("==================================================================\n");
    printf("This is a 2d simulation of Incremental Potential Contact Method.\n");
    printf("==================================================================\n");

    GLFWwindow* window = renderInit();
    if(!window)
        return -1;
    printf("window created.\n");

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    std::shared_ptr<world> world = world::GetWorldInstance();
    modelInit();

    world->physRegistration();

    auto startTime = std::chrono::high_resolution_clock::now();
    auto currentTime = startTime;
    auto previousTime = startTime;
    float elapsedTime = 0.0f;
    float dt = 0.0f;
    unsigned int totalFrameCount = 0;


    bool bShouldClose = false;
    printf("Simulation begins...\n");

    while(!bShouldClose)
    {
        currentTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - previousTime);
        unsigned int dt_ms = duration.count();
        dt = 0.001f * (dt_ms < 1 ? 1.0f : static_cast<float>(dt_ms));
        deltaTime = dt;//used for camera moving
        previousTime = currentTime;
        elapsedTime += dt;

        processInput(window);

        glClearColor(0.3f, 0.4f, 0.4f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //world->simulate(dt);
        world->simulate(0.004f);

        world->Draw(camera);

        glfwSwapBuffers(window);
        glfwPollEvents();
        bShouldClose = glfwWindowShouldClose(window);

        totalFrameCount++;
    }

    printf("Simulation ends.\n");
    glfwTerminate();
	return 0;
}
