cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)


project(ack)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(conan)
conan_cmake_run(CONANFILE conanfile.py
                BASIC_SETUP CMAKE_TARGETS
                BUILD missing)
				

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib) 
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib) 
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin) 

# add_subdirectory(lib/utilities)
# add_subdirectory(lib/meshDataLib)


add_executable(ack ack.cxx cabbie_curve_collection.cpp cabbie_curve_collection.h caliper.cpp caliper.h curve_collection.cpp curve_collection.h patrol.h patrol.cpp vacancy.cpp vacancy.h)
target_link_libraries(ack ${CMAKE_DL_LIBS} CONAN_PKG::boost CONAN_PKG::eigen)
target_include_directories(ack PRIVATE "${CMAKE_SOURCE_DIR}/lib")
