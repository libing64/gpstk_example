

find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

add_executable(gps_time gps_time.cpp)
target_link_libraries(gps_time gpstk)


