find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS, "include dir: ${EIGEN3_INCLUDE_DIR}")

add_executable(rinex2rtk rinex2rtk.cpp decorr.cpp)
target_link_libraries(rinex2rtk gpstk Eigen3::Eigen)

