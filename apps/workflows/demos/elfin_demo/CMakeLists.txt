cmake_minimum_required(VERSION 3.5)
project(elfin_demo)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/joint_goal_demo.py
  scripts/cart_goal_demo.py
  scripts/cart_path_goal_demo.py
  scripts/elfin_path_command.py
  scripts/get_robot_state.py
  scripts/get_enable_state.py
  scripts/elfin_enable_demo.py
  scripts/elfin_disable_demo.py
  scripts/stop_teleop_demo.py
  scripts/read_di_demo.py
  scripts/write_do_demo.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
