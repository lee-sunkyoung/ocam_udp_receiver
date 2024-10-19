cmake_minimum_required(VERSION 3.8)
project(udp_receiver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)


# Include directories
include_directories(
        include/
       ${rclcpp_INCLUDE_DIRS}
       ${sensor_msgs_INCLUDE_DIRS}
       ${cv_bridge_INCLUDE_DIRS}
       ${image_transport_INCLUDE_DIRS}
       ${OpenCV_INCLUDE_DIRS}  # OpenCV include directories 추가
)

# Declare the library 
add_library(${PROJECT_NAME}_lib
  src/udp_receiver.cpp
)

set(DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "image_transport"
  "cv_bridge"
)

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# Declare the executable
add_executable(udp_receiver src/main.cpp)
target_link_libraries(udp_receiver ${PROJECT_NAME}_lib)
ament_target_dependencies(udp_receiver ${DEPENDENCIES})

# Install
install(TARGETS
udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export include directories
ament_export_include_directories(include)

ament_package()

