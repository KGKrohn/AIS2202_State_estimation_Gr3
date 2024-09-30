cmake_minimum_required(VERSION 3.29)
project(AIS4104-assignments)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(Data_parsing)
add_subdirectory(estimation)
add_subdirectory(parameter_estimation)
add_subdirectory(state_estimation)


add_dependencies(state_estimation estimation)