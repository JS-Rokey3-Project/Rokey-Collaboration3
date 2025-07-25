# # cmake_minimum_required(VERSION 3.8)
# # project(grasp_executor_cpp)

# # if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# #   add_compile_options(-Wall -Wextra -Wpedantic)
# # endif()

# # # find dependencies
# # find_package(ament_cmake REQUIRED)
# # find_package(rclcpp REQUIRED)
# # find_package(geometry_msgs REQUIRED)
# # find_package(moveit_ros_planning_interface REQUIRED)
# # find_package(tf2_ros REQUIRED)
# # find_package(tf2_geometry_msgs REQUIRED)

# # add_executable(grasp_executor src/grasp_executor.cpp)
# # ament_target_dependencies(grasp_executor
# #   rclcpp
# #   geometry_msgs
# #   tf2_ros
# #   tf2_geometry_msgs
# #   moveit_ros_planning_interface
# # )

# # install(TARGETS
# #   grasp_executor
# #   DESTINATION lib/${PROJECT_NAME}
# # )

# # install(
# #   DIRECTORY launch
# #   DESTINATION share/${PROJECT_NAME}/
# # )

# # if(BUILD_TESTING)
# #   find_package(ament_lint_auto REQUIRED)
# #   # the following line skips the linter which checks for copyrights
# #   # comment the line when a copyright and license is added to all source files
# #   set(ament_cmake_copyright_FOUND TRUE)
# #   # the following line skips cpplint (only works in a git repo)
# #   # comment the line when this package is in a git repo and when
# #   # a copyright and license is added to all source files
# #   set(ament_cmake_cpplint_FOUND TRUE)
# #   ament_lint_auto_find_test_dependencies()
# # endif()

# # ament_package()
# cmake_minimum_required(VERSION 3.8)
# project(grasp_executor_cpp)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # Dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(control_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(aruco_msgs REQUIRED)

# # Executable
# add_executable(grasp_executor src/grasp_executor_aruco_auto.cpp)

# # Dependencies to target
# ament_target_dependencies(grasp_executor
#   rclcpp
#   rclcpp_action
#   control_msgs
#   geometry_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   moveit_ros_planning_interface
#   aruco_msgs  # ← 추가
# )

# # Install executable
# install(TARGETS
#   grasp_executor
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch directory
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# # Optional: lint / static analysis
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.8)
project(grasp_executor_cpp)

# 컴파일러 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # 🔹 추가

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}       # 🔹 추가
)

# 실행 파일
add_executable(grasp_executor src/grasp_executor_aruco_auto.cpp)

# 대상 실행 파일에 의존성 연결
ament_target_dependencies(grasp_executor
  rclcpp
  rclcpp_action
  control_msgs
  geometry_msgs
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  aruco_msgs
  Eigen3                          # 🔹 추가
)

# 설치 설정
install(TARGETS
  grasp_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 테스트용 린터 설정
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
