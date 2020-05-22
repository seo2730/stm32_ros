cmake_minimum_required(VERSION 2.8.3)
project(stm32_ros_encoder)

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

add_message_files(FILES motor.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
LIBRARIES stm32_ros_encoder
CATKIN_DEPENDS std_msgs roscpp message_runtime
)

# Uncomment if the package has a setup.py
#catkin_python_setup()

## 인클루드 디렉터리를 설정한다.
include_directories(${catkin_INCLUDE_DIRS})

## encoder 노드에 대한 빌드 옵션이다.
add_executable(encoder src/encoder.cpp)
add_dependencies(encoder ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(encoder ${catkin_LIBRARIES})

## motor 노드에 대한 빌드 옵션이다.
add_executable(velocity src/velocity.cpp)
add_dependencies(velocity ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity ${catkin_LIBRARIES})

## degree 노드에 대한 빌드 옵션이다.
add_executable(current src/current.cpp)
add_dependencies(current ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(current ${catkin_LIBRARIES})




