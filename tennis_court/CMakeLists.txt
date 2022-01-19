cmake_minimum_required(VERSION 3.5)
project(tennis_court)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Test REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallManagerStats.msg"
)

set (CMAKE_AUTOMOC ON)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS} ${Qt5Test_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${gazebo_dev_LIBRARY_DIRS} ${gazebo_ros_LIBRARY_DIRS})

add_library(gui_tennis_court_overlay SHARED ${headers_MOC} src/GUITennisCourtOverlay.cc)
rosidl_target_interfaces(gui_tennis_court_overlay ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gui_tennis_court_overlay ${GAZEBO_LIBRARIES}
  ${gazebo_ros_LIBRARIES}
  ${gazebo_dev_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${Qt5Test_LIBRARIES})
ament_export_libraries(gui_tennis_court_overlay)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(scripts/)

ament_package()

install(TARGETS
    gui_tennis_court_overlay
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
    scripts/ball_manager.py
    DESTINATION lib/${PROJECT_NAME}/
)

set(
  AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_tennis_court
  "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
