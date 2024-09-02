cmake_minimum_required(VERSION 3.8)
project(arussim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

include_directories(include)

# generate executables
add_executable(arussim_exec src/arussim_node.cpp)
ament_target_dependencies(arussim_exec rclcpp std_msgs custom_msgs)

  
install(TARGETS
  arussim_exec
  DESTINATION lib/${PROJECT_NAME})

# share launchs location
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
