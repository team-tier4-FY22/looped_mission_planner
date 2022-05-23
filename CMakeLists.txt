cmake_minimum_required(VERSION 3.8)
project(looped_mission_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LOOPED_MISSION_PLANNER_SRC
  src/looped_mission_planner_core.cpp)

set(LOOPED_MISSION_PLANNER_HEADERS
  include/looped_mission_planner/looped_mission_planner_core.hpp)

ament_auto_add_executable(${PROJECT_NAME}
  src/looped_mission_planner_node.cpp
  ${LOOPED_MISSION_PLANNER_SRC}
  ${LOOPED_MISSION_PLANNER_HEADERS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
