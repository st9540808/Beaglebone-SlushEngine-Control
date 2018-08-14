cmake_minimum_required(VERSION 3.4)

project(Servo C CXX)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-DDEBUG=1) # enable debug print

file(GLOB SOURCE src/*.c src/*.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} STATIC ${SOURCE})
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

# test
# add_executable(servo Servo.cpp)
# set_target_properties(servo PROPERTIES LINKER_LANGUAGE CXX)
# target_link_libraries(servo PWM)

