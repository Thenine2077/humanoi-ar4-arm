cmake_minimum_required(VERSION 3.8)
project(ar_dual)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf srdf launch config meshes DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/ar_calibrate.py scripts/ar_ports.py scripts/flash_teensy.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
