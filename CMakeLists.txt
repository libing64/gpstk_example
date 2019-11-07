cmake_minimum_required(VERSION 3.1)
project(example)

find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

add_executable(example example.cpp)
target_link_libraries(example gpstk)