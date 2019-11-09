find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

add_executable(rinex2 rinex2.cpp)
target_link_libraries(rinex2 gpstk)


add_executable(rinex3 rinex3.cpp)
target_link_libraries(rinex3 gpstk)
