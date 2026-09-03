cmake_minimum_required(VERSION 3.14)
project(oriented_field_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

# --- ROS 非依存のマッチングライブラリ ---
add_library(oriented_field_matcher STATIC src/oriented_field_matcher.cpp)
target_include_directories(oriented_field_matcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(oriented_field_matcher ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(oriented_field_matcher OpenMP::OpenMP_CXX)
endif()

# --- ROS 2 ノード ---
add_executable(ofl_node src/ofl_node.cpp)
target_include_directories(ofl_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(ofl_node
  rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs tf2_ros
)
target_link_libraries(ofl_node oriented_field_matcher ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(ofl_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS ofl_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

include(CTest)

# 各処理段の入出力不変条件を検査する ROS 非依存の単体テスト。
if(BUILD_TESTING)
  add_executable(test_matcher tests/test_matcher.cpp)
  target_link_libraries(test_matcher oriented_field_matcher ${OpenCV_LIBS})
  target_include_directories(test_matcher PRIVATE include)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(test_matcher OpenMP::OpenMP_CXX)
  endif()
  add_test(NAME matcher_invariants COMMAND test_matcher)

  # AMCL 監視の判定。ヘッダだけなので OpenCV も ROS も要らない。
  add_executable(test_reseed_policy tests/test_reseed_policy.cpp)
  target_include_directories(test_reseed_policy PRIVATE include)
  add_test(NAME reseed_policy COMMAND test_reseed_policy)
endif()

ament_package()
