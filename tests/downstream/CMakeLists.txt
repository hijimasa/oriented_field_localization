cmake_minimum_required(VERSION 3.14)
project(oriented_field_localization_downstream_smoke LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
find_package(oriented_field_localization REQUIRED)

add_executable(downstream_smoke consumer.cpp)
target_link_libraries(downstream_smoke PRIVATE
  oriented_field_localization::oriented_field_matcher
  oriented_field_localization::map_yaml_loader
)
