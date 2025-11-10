cmake_minimum_required(VERSION 3.10.2)
project(nova_description)
find_package(ament_cmake REQUIRED)


install(DIRECTORY urdf meshes launch worlds config DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/ros_gz_${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/ros_gz_${PROJECT_NAME}.sh.in")
ament_package()
