cmake_minimum_required(VERSION 3.22)

project(ipc2dsim)

add_executable(${PROJECT_NAME} dev.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
	CXX_STANDARD 11
	CXX_STANDARD_REQUIRED YES)

target_sources(${PROJECT_NAME}
	PRIVATE
		Object.h
		world.h
		modelInit.h
		utils.h
		Energy/elasticity.h
		Energy/gravity.h
		Energy/inertia.h
		Energy/sparseMatrix.h
		Energy/spring.h
		Energy/barrier.h
		phyScene.h
		neoHookeanCalc.h
		repulsionCalc.h
)

target_sources(${PROJECT_NAME}
	PRIVATE
		Object.cpp
		world.cpp
		modelInit.cpp
		Energy/gravity.cpp
		phyScene.cpp
		neoHookeanCalc.cpp
		repulsionCalc.cpp
)

target_link_directories(${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/eigen-3.4.0
		${CMAKE_CURRENT_SOURCE_DIR}/Energy
)

find_package(Catch2 REQUIRED PATHS /opt/homebrew/lib/cmake/Catch2)

target_link_libraries(${PROJECT_NAME} Catch2::Catch2 Catch2::Catch2WithMain)

target_include_directories(${PROJECT_NAME} PRIVATE /opt/homebrew/include)


if(APPLE OR WIN32)

	find_package(OpenGL REQUIRED)
	find_library(glfwLIB NAMES glfw glfw3 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)

	target_include_directories(${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include
			${CMAKE_CURRENT_SOURCE_DIR}/include/glfw
			${CMAKE_CURRENT_SOURCE_DIR}/include/glad
	)

	target_sources(${PROJECT_NAME}
		PRIVATE
			glad.c
			renderinit.cpp
			shader.cpp
	)


	target_sources(${PROJECT_NAME}
		PRIVATE
			stb_image.h
			renderinit.h
			macros.h
			camera.h
			shader.h
	)

	target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARY} ${glfwLIB})

endif()

if(APPLE)
	find_library(COCOA_LIBRARY Cocoa)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(COREVIDEO_LIBRARY CoreVideo)
	target_link_libraries(${PROJECT_NAME}
			"${COCOA_LIBRARY}"
			"${IOKIT_LIBRARY}"
			"${COREVIDEO_LIBRARY}"
	)
endif()

