find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

add_executable(rinex2rtk rinex2rtk.cpp)
target_link_libraries(rinex2rtk gpstk)

