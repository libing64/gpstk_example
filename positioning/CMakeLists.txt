find_package(GPSTK)
include_directories(${GPSTK_INCLUDE_DIRS})

add_executable(raim_example raim_example.cpp)
target_link_libraries(raim_example gpstk)

add_executable(PRsolve_example PRsolve_example.cpp)
target_link_libraries(PRsolve_example gpstk)

