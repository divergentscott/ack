cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)


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


add_executable(ack ack.cxx cabbie_curve_collection.cpp cabbie_curve_collection.h zone_surveyor.cpp zone_surveyor.h curve_collection.cpp curve_collection.h zoning_commisioner.cpp zoning_commisioner.h zoning_cartographer_svg.cpp zoning_cartographer_svg.h zoning_board.cpp zoning_board.h)
target_link_libraries(ack ${CMAKE_DL_LIBS} PUBLIC CONAN_PKG::boost CONAN_PKG::eigen CONAN_PKG::VTK-mini)
target_include_directories(ack PRIVATE "${CMAKE_SOURCE_DIR}/lib")
