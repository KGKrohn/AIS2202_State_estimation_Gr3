find_package(Eigen3 CONFIG REQUIRED)

add_library(estimation STATIC kalman_filter.cpp include/estimation/kalman_filter.h)

target_link_libraries(estimation PRIVATE Eigen3::Eigen)
target_include_directories(estimation PUBLIC include/)