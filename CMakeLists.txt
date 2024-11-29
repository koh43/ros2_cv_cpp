cmake_minimum_required(VERSION 3.8)
project(ros2_cv_cpp)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
# Add your source files
add_library(cam_utils
  src/cam_utils.cpp
)

# Define the executables
add_executable(talker src/talker.cpp)
add_executable(listener src/listener.cpp)
add_executable(pub_cam src/pub_cam.cpp)

target_link_libraries(pub_cam cam_utils ${OpenCV_LIBS})
# Add ROS 2 dependencies
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(pub_cam rclcpp sensor_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  pub_cam
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cam_utils
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(
  include
)

ament_package()
