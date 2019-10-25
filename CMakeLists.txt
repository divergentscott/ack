cmake_minimum_required(VERSION 2.6)

project(ack)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include(conan)
conan_cmake_run(CONANFILE conanfile.py
                BASIC_SETUP CMAKE_TARGETS
                BUILD missing)

# include_directories(${CMAKE_SOURCE_DIR})

add_executable(ack ack.cxx)
target_link_libraries(ack ${CMAKE_DL_LIBS} CONAN_PKG::boost_graph CONAN_PKG::tetgen)

add_executable(quick_tour quick_tour.cpp)
target_link_libraries(quick_tour ${CMAKE_DL_LIBS} CONAN_PKG::boost_graph)