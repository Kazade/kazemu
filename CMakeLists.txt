project(kazemu)
cmake_minimum_required(VERSION 2.8)

ADD_DEFINITIONS("-Wall -g")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})

