find_package(Eigen3 CONFIG REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_executable(parameter_estimation main.cpp ../Data_parsing/Calibration.hpp)

target_link_libraries(parameter_estimation PUBLIC Eigen3::Eigen)
target_include_directories(parameter_estimation PRIVATE ${RAPIDCSV_INCLUDE_DIRS})
