find_package(Eigen3 CONFIG REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

#add_library(estimation STATIC kalman_filter.cpp include/estimation/kalman_filter.h ../Data_parsing/DataStore.hpp)

add_executable(estimation kalman_filter.cpp
        include/estimation/kalman_filter.h
        ../Data_parsing/Baseline_orientations.hpp
        ../Data_parsing/Baseline_wrench.hpp
        ../Data_parsing/BaselineAcc.hpp)

target_include_directories(estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})
target_link_libraries(estimation PUBLIC Eigen3::Eigen)
target_include_directories(estimation PRIVATE include/)
#target_link_libraries(estimation PRIVATE Eigen3::Eigen)
#target_include_directories(estimation PUBLIC include/)