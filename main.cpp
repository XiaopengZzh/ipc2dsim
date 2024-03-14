
#include "renderinit.h"
#include "modelInit.h"
#include "world.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "utils.h"

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

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    std::shared_ptr<world> world = world::GetWorldInstance();
    modelInit();

    bool bShouldClose = false;
    printf("Simulation begins...\n");

    while(!bShouldClose)
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glClearColor(0.3f, 0.4f, 0.4f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        world->Draw(camera);

        /*
        world->objList[0].vertices[0] = glm::vec2(-1.0f, 1.0f) + glm::vec2(glm::cos(currentFrame), glm::sin(currentFrame));
        world->objList[0].vertices[1] = glm::vec2(-1.0f, -1.0f) + glm::vec2(glm::cos(currentFrame), glm::sin(currentFrame));
        world->objList[0].vertices[2] = glm::vec2(1.0f, 1.0f) + glm::vec2(glm::cos(currentFrame), glm::sin(currentFrame));
        world->objList[0].updateVertexBuffer();
         */

        glfwSwapBuffers(window);
        glfwPollEvents();
        bShouldClose = glfwWindowShouldClose(window);
    }

    printf("Simulation ends.\n");
    glfwTerminate();
	return 0;
}
