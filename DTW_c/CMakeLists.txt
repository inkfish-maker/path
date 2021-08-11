cmake_minimum_required(VERSION 3.0.2)
project(path_plan)

################################################
### 设置编译器标准为C++14,编译类型为Release
################################################
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14 -fopenmp)

################################################
### 设置工作根目录
################################################
set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
set(NODE_NAME ${PROJECT_NAME})
configure_file(
  ${PROJECT_SOURCE_DIR}/include/PATH.h.in
  ${PROJECT_SOURCE_DIR}/include/PATH.h
)

message( STATUS CMAKE_PREFIX_PATH )

#set opencv version
set(CMAKE_PREFIX_PATH "/usr/local/opencv/3.4.14")
find_package(OpenCV  REQUIRED)
#显示opencv版本
MESSAGE(STATUS "OpenCV version: "${OpenCV_VERSION})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/include)

################################################
### 将库文件存入TARGET_LIBRARIES变量
################################################
set(TARGET_LIBRARIES "")
list(APPEND TARGET_LIBRARIES ${catkin_LIBRARIES})
list(APPEND TARGET_LIBRARIES ${OpenCV_LIBS}) # opencv 库

################################################
###  可执行节点
################################################
add_executable(${PROJECT_NAME}_node  src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_node  ${TARGET_LIBRARIES})

################################################
###  安装
################################################
# INSTALL(PROGRAMS script/create_cone.sh DESTINATION bin)
# INSTALL(PROGRAMS script/output_file.sh DESTINATION bin)
