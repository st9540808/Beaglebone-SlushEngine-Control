cmake_minimum_required(VERSION 3.4)

project(PWM C)
add_definitions(-DDEBUG=1) # enable debug print

file(GLOB SOURCE src/*.c)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} STATIC ${SOURCE})

