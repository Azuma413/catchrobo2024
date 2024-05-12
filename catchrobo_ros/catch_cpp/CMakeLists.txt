cmake_minimum_required(VERSION 3.8)
project(catch_cpp)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
src/dynamixel_workbench_toolbox/dynamixel_driver.cpp include/dynamixel_workbench_toolbox/dynamixel_driver.h
src/dynamixel_workbench_toolbox/dynamixel_item.cpp include/dynamixel_workbench_toolbox/dynamixel_item.h
src/dynamixel_workbench_toolbox/dynamixel_tool.cpp include/dynamixel_workbench_toolbox/dynamixel_tool.h
src/dynamixel_workbench_toolbox/dynamixel_workbench.cpp include/dynamixel_workbench_toolbox/dynamixel_workbench.h
)

ament_auto_add_executable(read_serial_node src/read_serial_node.cpp)
ament_auto_add_executable(finger_control_node src/finger_control_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()