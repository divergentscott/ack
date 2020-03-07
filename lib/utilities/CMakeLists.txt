
project(utilities)

#set_property(SOURCE ${GIT_POST_CONFIGURE_FILE} PROPERTY GENERATED 1)

add_library(${PROJECT_NAME} STATIC 
                                options.h 
                                options.cpp 
                                d3derr.h 
                                d3derr.cpp 
                                ioHelpers.h 
                                ioHelpers.cpp )

target_link_libraries(${PROJECT_NAME} PUBLIC CONAN_PKG::boost)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_BINARY_DIR})

if (GCOV_COVERAGE AND UNIX AND NOT APPLE)
	target_link_libraries(${PROJECT_NAME} PUBLIC gcov)
endif()

