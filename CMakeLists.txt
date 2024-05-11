cmake_minimum_required(VERSION 3.22)

project(ipc2dsim)

add_executable(${PROJECT_NAME} dev.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES
	CXX_STANDARD 17
	CXX_STANDARD_REQUIRED YES)

target_sources(${PROJECT_NAME}
	PRIVATE
		Object.h
		world.h
		modelInit.h
		utils.h
		phyScene.h
		neoHookeanCalc.h
		repulsionCalc.h
)

target_sources(${PROJECT_NAME}
	PRIVATE
		Object.cpp
		world.cpp
		modelInit.cpp
		phyScene.cpp
		neoHookeanCalc.cpp
		repulsionCalc.cpp
)

# find_package(Catch2 REQUIRED PATHS /opt/homebrew/lib/cmake/Catch2)

find_package(Catch2 REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC {CATCH2_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME} Catch2::Catch2 Catch2::Catch2WithMain)

# target_include_directories(${PROJECT_NAME} PRIVATE /opt/homebrew/include)

find_package(Eigen3 REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})

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

target_include_directories(${PROJECT_NAME} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion)

target_sources(${PROJECT_NAME} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/avx.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/avx.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/ccd.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/ccd.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/interval_root_finder.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/interval_root_finder.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/interval.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/interval.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/timer.hpp
		${CMAKE_CURRENT_SOURCE_DIR}/tight_inclusion/types.hpp
)