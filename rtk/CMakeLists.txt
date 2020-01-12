find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS, "include dir: ${EIGEN3_INCLUDE_DIR}")

add_executable(rinex2rtk rinex2rtk.cpp rtk.cpp decorr.cpp pvt.cpp)
target_link_libraries(rinex2rtk gpstk Eigen3::Eigen)


add_executable(pvt_example pvt_example.cpp decorr.cpp pvt.cpp)
target_link_libraries(pvt_example gpstk Eigen3::Eigen)

add_executable(dgps_example dgps_example.cpp rtk.cpp decorr.cpp pvt.cpp)
target_link_libraries(dgps_example gpstk Eigen3::Eigen)


# add_executable(ppp_example ppp_example.cpp decorr.cpp pvt.cpp)
# target_link_libraries(ppp_example gpstk Eigen3::Eigen)

