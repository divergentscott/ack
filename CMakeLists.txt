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

add_subdirectory(lib/utilities)
add_subdirectory(lib/meshDataLib)


add_executable(ack ack.cxx)
target_link_libraries(ack ${CMAKE_DL_LIBS} CONAN_PKG::boost utilities meshDataLib )
target_include_directories(ack PRIVATE "${CMAKE_SOURCE_DIR}/lib")

#Set properties on a target. 
#We use this here to set -DTETLIBRARY for when compiling the
#library
#set_target_properties(tet PROPERTIES "COMPILE_DEFINITIONS" TETLIBRARY)a
#add_definitions(-DTETLIBRARY)

# include_directories(${CMAKE_SOURCE_DIR})



# add_executable(cra cra.cpp)
#target_include_directories(cra PUBLIC "C:/Users/sscott/Programs/mmg/build/include")
# target_link_libraries(cra CONAN_PKG::mmg)
#target_link_libraries(cra ${CMAKE_DL_LIBS} "C:/Users/sscott/Programs/mmg/build/lib/Debug/mmg.lib" "C:/Users/sscott/Programs/mmg/build/lib/Debug/mmg2d.lib" "C:/Users/sscott/Programs/mmg/build/lib/Debug/mmg3d.lib" "C:/Users/sscott/Programs/mmg/build/lib/Debug/mmgs.lib")

#find_package(Mmg)
#message(STATUS "!!!! all conan libs is this:  ${CONAN_LIBS}")


#message(STATUS "!!!! conan root is this: ${CONAN_MMG_ROOT}")
#message(STATUS "!!!! conan lib dirs is this: ${CONAN_LIB_DIRS_MMG}")
#message(STATUS "!!!! conan libs is this: ${CONAN_LIBS_MMG}")
#message(STATUS "!!!! conan pkg libs is this: ${CONAN_PKG_LIBS_MMG}")

#add_executable(quick_tour quick_tour.cpp)
#target_link_libraries(quick_tour ${CMAKE_DL_LIBS} CONAN_PKG::boost_graph)

