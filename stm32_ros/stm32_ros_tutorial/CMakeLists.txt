cmake_minimum_required(VERSION 2.8.3)
project(stm32_ros_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
LIBRARIES stm32_ros_tutorial
CATKIN_DEPENDS std_msgs roscpp
)

# Uncomment if the package has a setup.py
#catkin_python_setup()

## 인클루드 디렉터리를 설정한다.
include_directories(${catkin_INCLUDE_DIRS})

## tutorial_subscriber 노드에 대한 빌드 옵션이다.
add_executable(tutorial_subscriber src/tutorial_subscriber.cpp)
add_dependencies(tutorial_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(tutorial_subscriber ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS src/led.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



