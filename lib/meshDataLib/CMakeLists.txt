
project(meshDataLib)

add_library(${PROJECT_NAME} STATIC meshData.cpp meshData.h bdfIO.cpp bdfIO.h io.cpp io.h utils.h utils.cpp vertexCluster.h vertexCluster.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/lib")
message(STATUS "Include: ${CMAKE_SOURCE_DIR}/lib")
target_link_libraries(${PROJECT_NAME} PUBLIC CONAN_PKG::VTK-mini CONAN_PKG::boost utilities)

if (GCOV_COVERAGE AND UNIX AND NOT APPLE)
	target_link_libraries(${PROJECT_NAME} PUBLIC gcov)
endif()

