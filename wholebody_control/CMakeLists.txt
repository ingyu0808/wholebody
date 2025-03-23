cmake_minimum_required(VERSION 3.8)
project(wholebody_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(TBB REQUIRED)

set(Franka_DIR "~/libfranka/build")
find_package(Franka REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(franka_ex src/franka_ex.cpp src/examples_common.cpp)
add_executable(franka_control src/franka_control.cpp src/examples_common.cpp)
add_executable(franka_rl_control src/franka_rl_control.cpp src/examples_common.cpp)

target_link_libraries(franka_control TBB::tbb)
target_link_libraries(franka_rl_control TBB::tbb)

ament_target_dependencies(franka_ex 
  Franka
  Eigen3
)

ament_target_dependencies(franka_control 
  Franka
  Eigen3
  rclcpp
  std_msgs
)
ament_target_dependencies(franka_rl_control 
  Franka
  Eigen3
  rclcpp
  std_msgs
)

install(TARGETS franka_ex franka_control franka_rl_control
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
