cmake_minimum_required(VERSION 3.4)

project(SPI C)

file(GLOB SOURCE SPI_Library/SPI.c)
include_directories(${PROJECT_SOURCE_DIR}/SPI_Library)

add_library(${PROJECT_NAME} STATIC ${SOURCE})
target_link_libraries(${PROJECT_NAME})
set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/SPI_Library
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
