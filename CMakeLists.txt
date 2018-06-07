cmake_minimum_required(VERSION 3.4)

project(Robot CXX)
set(CMAKE_CXX_STANDARD 11)

file(GLOB SOURCE *.cpp)
file(GLOB HEADER *.h)

add_subdirectory(BeagleBone-SPI-Library)
add_subdirectory(lib-l6470)

add_executable(Robot ${SOURCE} ${HEADER})
include_directories(${l6470_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} l6470)
