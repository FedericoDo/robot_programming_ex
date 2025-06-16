cmake_minimum_required(VERSION 3.8)
project(robotic_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)  
find_package(visualization_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include_directories(include /usr/include/eigen3)

add_executable(publisher src/robotic_arm_node.cpp 
                  src/dh_params.cpp) 
ament_target_dependencies(publisher rclcpp std_msgs yaml-cpp ament_index_cpp visualization_msgs)

target_link_libraries(publisher yaml-cpp)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)
        
install(TARGETS
        publisher
        DESTINATION lib/${PROJECT_NAME})


ament_package()
