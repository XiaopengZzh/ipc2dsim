
#include "renderinit.h"
#include "modelInit.h"
#include "world.h"

#include "eigen-3.4.0/Eigen/Dense"
#include <chrono>

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

bool bUnitTest = false;
bool bRebounded = false;
unsigned int checkedIdx = 1;

extern Camera camera;
extern float deltaTime, lastFrame;

TEST_CASE("dev", "[development]")
{
    bUnitTest = false;

    printf("==================================================================\n");
    printf("This is a 2d simulation of Incremental Potential Contact Method.\n");
    printf("==================================================================\n");

    GLFWwindow* window = renderInit();
    //if(!window)
        //return -1;
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

        printf("### timestep : %d\n", totalFrameCount);
        //world->simulate(dt);
        world->simulate(0.01f);

        world->Draw(camera);

        glfwSwapBuffers(window);
        glfwPollEvents();
        bShouldClose = glfwWindowShouldClose(window);

        totalFrameCount++;
    }

    printf("Simulation ends.\n");
    glfwTerminate();
	//return 0;
}


TEST_CASE("1tri1cube", "[unit tests]")
{
    /*
    printf("==================================================================\n");
    printf("This is a 2d simulation of Incremental Potential Contact Method.\n");
    printf("==================================================================\n");
    */

    bUnitTest = true;
    checkedIdx = 4;

    GLFWwindow* window = renderInit();
    //if(!window)
    //return -1;
    //printf("window created.\n");

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    std::shared_ptr<world> world = world::GetWorldInstance();

    modelInit_utest("../model/1tri1cube.xyz");

    //modelInit_utest("../model/2cubes.xyz");
    //modelInit_utest("../model/2triangles.xyz");



    world->physRegistration();

    auto startTime = std::chrono::high_resolution_clock::now();
    auto currentTime = startTime;
    auto previousTime = startTime;
    float elapsedTime = 0.0f;
    float dt = 0.0f;
    unsigned int totalFrameCount = 0;


    bool bShouldClose = false;
    //printf("Simulation begins...\n");

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

        //printf("### timestep : %d\n", totalFrameCount);
        world->simulate(0.01f);

        world->Draw(camera);

        glfwSwapBuffers(window);
        glfwPollEvents();
        bShouldClose = glfwWindowShouldClose(window);

        totalFrameCount++;

        if(elapsedTime > 3.5f)
        {
            bShouldClose = true;
        }
    }
    CHECK(bRebounded);
    //printf("Simulation ends.\n");
    glfwTerminate();
    //return 0;
}

TEST_CASE("2cubes", "[unit tests]")
{
    /*
    printf("==================================================================\n");
    printf("This is a 2d simulation of Incremental Potential Contact Method.\n");
    printf("==================================================================\n");
    */

    bUnitTest = true;
    checkedIdx = 5;

    GLFWwindow* window = renderInit();
    //if(!window)
    //return -1;
    //printf("window created.\n");

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    std::shared_ptr<world> world = world::GetWorldInstance();
    world->clear();
    //modelInit_utest("../model/1tri1cube.xyz");

    modelInit_utest("../model/2cubes.xyz");
    //modelInit_utest("../model/2triangles.xyz");



    world->physRegistration();

    auto startTime = std::chrono::high_resolution_clock::now();
    auto currentTime = startTime;
    auto previousTime = startTime;
    float elapsedTime = 0.0f;
    float dt = 0.0f;
    unsigned int totalFrameCount = 0;


    bool bShouldClose = false;
    //printf("Simulation begins...\n");

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

        //printf("### timestep : %d\n", totalFrameCount);
        world->simulate(0.01f);

        world->Draw(camera);

        glfwSwapBuffers(window);
        glfwPollEvents();
        bShouldClose = glfwWindowShouldClose(window);

        totalFrameCount++;

        if(elapsedTime > 3.5f)
        {
            bShouldClose = true;
        }
    }
    CHECK(bRebounded);
    //printf("Simulation ends.\n");
    glfwTerminate();
    //return 0;
}

TEST_CASE("2triangles", "[unit tests]")
{
    /*
    printf("==================================================================\n");
    printf("This is a 2d simulation of Incremental Potential Contact Method.\n");
    printf("==================================================================\n");
    */

    bUnitTest = true;
    checkedIdx = 3;

    GLFWwindow* window = renderInit();
    //if(!window)
    //return -1;
    //printf("window created.\n");

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    std::shared_ptr<world> world = world::GetWorldInstance();
    world->clear();
    //modelInit_utest("../model/1tri1cube.xyz");

    //modelInit_utest("../model/2cubes.xyz");
    modelInit_utest("../model/2triangles.xyz");



    world->physRegistration();

    auto startTime = std::chrono::high_resolution_clock::now();
    auto currentTime = startTime;
    auto previousTime = startTime;
    float elapsedTime = 0.0f;
    float dt = 0.0f;
    unsigned int totalFrameCount = 0;


    bool bShouldClose = false;
    //printf("Simulation begins...\n");

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

        //printf("### timestep : %d\n", totalFrameCount);
        world->simulate(0.01f);

        world->Draw(camera);

        glfwSwapBuffers(window);
        glfwPollEvents();
        bShouldClose = glfwWindowShouldClose(window);

        totalFrameCount++;

        if(elapsedTime > 3.5f)
        {
            bShouldClose = true;
        }
    }
    CHECK(bRebounded);
    //printf("Simulation ends.\n");
    glfwTerminate();
    //return 0;
}