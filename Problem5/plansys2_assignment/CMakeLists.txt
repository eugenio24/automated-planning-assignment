cmake_minimum_required(VERSION 3.5)
project(plansys2_assignment)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    plansys2_msgs
    plansys2_executor
)

add_executable(empty_action_node src/empty_action_node.cpp)
ament_target_dependencies(empty_action_node ${dependencies})

add_executable(fill_action_node src/fill_action_node.cpp)
ament_target_dependencies(fill_action_node ${dependencies})

add_executable(load_empty_action_node src/load_empty_action_node.cpp)
ament_target_dependencies(load_empty_action_node ${dependencies})

add_executable(load_filled_action_node src/load_filled_action_node.cpp)
ament_target_dependencies(load_filled_action_node ${dependencies})

add_executable(move_carrier_to_warehouse_action_node src/move_carrier_to_warehouse_action_node.cpp)
ament_target_dependencies(move_carrier_to_warehouse_action_node ${dependencies})

add_executable(move_carrier_to_workstations_action_node src/move_carrier_to_workstations_action_node.cpp)
ament_target_dependencies(move_carrier_to_workstations_action_node ${dependencies})

add_executable(move_to_carrier_action_node src/move_to_carrier_action_node.cpp)
ament_target_dependencies(move_to_carrier_action_node ${dependencies})

add_executable(unload_empty_action_node src/unload_empty_action_node.cpp)
ament_target_dependencies(unload_empty_action_node ${dependencies})

add_executable(unload_filled_action_node src/unload_filled_action_node.cpp)
ament_target_dependencies(unload_filled_action_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  empty_action_node
  fill_action_node
  load_empty_action_node
  load_filled_action_node
  move_carrier_to_warehouse_action_node
  move_carrier_to_workstations_action_node
  move_to_carrier_action_node
  unload_empty_action_node
  unload_filled_action_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()