cmake_minimum_required(VERSION 3.8)
project(arussim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# generate executables
add_executable(arussim_exec src/arussim_node.cpp)
ament_target_dependencies(arussim_exec rclcpp 
                            std_msgs 
                            geometry_msgs 
                            sensor_msgs 
                            visualization_msgs 
                            custom_msgs 
                            tf2_ros
                            PCL
                            pcl_conversions)

  
install(TARGETS
  arussim_exec
  DESTINATION lib/${PROJECT_NAME})

# share folders location
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})


ament_package()
