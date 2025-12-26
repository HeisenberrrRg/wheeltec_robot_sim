cmake_minimum_required(VERSION 3.8)
project(wheeltec_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装规则：把 meshes, urdf, launch 文件夹安装到系统
install(DIRECTORY
  launch
  meshes
  urdf
  world
  maps    
  params
  rvizconfig
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
