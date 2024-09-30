cmake_minimum_required(VERSION 3.5)
project(plansys2_project)

# Use ament_cmake and ament_python
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)      # For building the Python package
find_package(rclpy REQUIRED)               # For Python nodes
find_package(plansys2_msgs REQUIRED)       # PlanSys2 messages (for Python package)
find_package(plansys2_executor REQUIRED)   # PlanSys2 executor (for Python package)

# Set dependencies
set(dependencies
    rclcpp
    rclpy
    plansys2_msgs
    plansys2_executor
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python action nodes (scripts)
install(PROGRAMS
  plansys2_project/fillbox_action_node.py
  plansys2_project/pickupbox_action_node.py
  plansys2_project/loadcarrier_action_node.py
  plansys2_project/move_action_node.py
  plansys2_project/deliver_action_node.py
  plansys2_project/emptybox_action_node.py
  plansys2_project/return_action_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install non-code resources like launch and PDDL files
install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

# Testing (if enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

# Export dependencies
ament_export_dependencies(${dependencies})

ament_package()
