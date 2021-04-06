# CMake要求本版信息即项目名称，创建时自动生成
cmake_minimum_required(VERSION 3.12)
project(useEigen)

# 申明采用C++11标准，项目创建时自动生成
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  
# 寻找Eigen库
find_package(Eigen3 REQUIRED)
# 将Eigen库include进来
include_directories("/usr/include/Eigen")

message("${PROJECT_SOURCE_DIR}")

#cmake 设置debug模式
set(CMAKE_BUILD_TYPE "Debug")
 
add_executable(wall_obstacle_data  ./config/global_variable.cpp ./src/wall_obstacle_data.cpp ./config/read_config.cpp  ./src/base_lib_code.cpp   ./src/distance_compute/distance_compute.cpp)

add_executable(transition_possibility_analysis  ./config/global_variable.cpp  ./config/read_config.cpp src/transition_possibility_analysis.cpp ./src/base_lib_code.cpp ./src/distance_compute/distance_compute.cpp)


target_link_libraries(wall_obstacle_data)
target_link_libraries(transition_possibility_analysis)
# add_executable (wall_transitional_cost_fun  ./src/wall_transitional_cost_fun.cpp  ./config/global_variable.cpp)
#add_executable(read_config  ./config/read_config.cpp)




## 需要修改路径的地方： CMakeList.txt | 各包含了 read_config.h 的文件 
