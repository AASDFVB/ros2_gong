cmake_minimum_required(VERSION 3.8)
project(robotarm)

find_package(ament_cmake REQUIRED)

# 纯资源包：urdf、mesh、rviz 配置、launch、config 全部原样安装到 share/
install(DIRECTORY
    config
    launch
    meshes
    rviz
    urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
