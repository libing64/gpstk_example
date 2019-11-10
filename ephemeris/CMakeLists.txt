find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

add_executable(ephemeris_example ephemeris_example.cpp)
target_link_libraries(ephemeris_example gpstk)

