find_package(Eigen3 CONFIG REQUIRED)
find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

add_executable(Data_parsing main.cpp
        DataStore.cpp
        DataStore.hpp)

target_link_libraries(Data_parsing PUBLIC Eigen3::Eigen)
target_include_directories(Data_parsing PRIVATE ${RAPIDCSV_INCLUDE_DIRS})
target_include_directories(Data_parsing PUBLIC "${CMAKE_SOURCE_DIR}/Data")
