cmake_minimum_required(VERSION 3.4)

project(l6470 CXX)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-DDEBUG=1) # enable debug print

file(GLOB SOURCE src/*.cpp)

include_directories(${SPI_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_link_libraries(${PROJECT_NAME} SPI)

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
